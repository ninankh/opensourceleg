"""Home the VSO, zero the ankle encoder, then move slider and record on demand.

Console commands (type and press Enter):
    <number>   move slider to this stiffness (0-100 %), only when not recording
    r          start recording (new trial index)
    s          stop recording
    z          re-zero ankle encoder at the current pose (not while recording)
    q          return to 0 % and quit
"""

import queue
import threading
import time

import numpy as np

from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.vso import VSO
from opensourceleg.sensors.adc import ADS114S0x, ChannelConfig
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.sensors.hall import DRV5056
import lgpio

MUX0_PIN = 35
MUX1_PIN = 39
RP1_GPIOCHIP = 0

BAT_0 = 9.9 # V 
BAT_1 = 12.6 # V
ADC_0 = 0.318 # V
ADC_1 = 3.296 # V

TIME_TO_STEP = 1.0
FREQUENCY = 200  # loop rate (Hz)
DT = 1 / FREQUENCY
READ_FREQUENCY = 10  # console readout rate (Hz)
OFFLINE = False
SIDE = -1  # 1: left-leg lateral encoder (-1 if medial); -1: right-leg lateral
ENCODER_ALPHA = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)
ZERO_SAMPLES = 100  # encoder samples averaged when zeroing
STIFFNESS_MIN = 0.5
STIFFNESS_MAX = 99.5

# Choose which sensors are used, logged, and printed.
ENABLE_HALL = True


def home_motor(actuator: MaxonActuator, encoder_counter: LS7366R) -> None:
    """Home the provided actuator using the given encoder counter."""
    homing_logger = Logger(log_path="./logs", file_name="home_motor")

    actuator.set_motor_encoder(encoder_counter)
    actuator.position_control_config()

    if actuator._is_homed:
        homing_logger.info("Actuator already homed to 0% stiffness.")
    else:
        homing_logger.info("Actuator homing to 0% stiffness.")
        actuator.home(home_zero=False,homing_pwm=0.45)
        encoder_counter.clear_counter()
        actuator._is_homed = True
        

        actuator.update()
        actuator.check_coupler_drift()
        homing_logger.info(
            f"Homing complete. position = "
            f"{actuator.cts_to_perc(actuator.motor_position_cts):.2f}% "
            f"({actuator.motor_position_mm:.3f} mm)"
        )

def _clamp_target_cts(actuator: MaxonActuator, desired_cts: int) -> int:
    """Clamp a target position (counts) into the safe slider range."""
    desired_mm = actuator.cts_to_mm(desired_cts)
    if desired_mm > actuator.slider_max_mm:
        return actuator.slider_max_counts
    if desired_mm < actuator.slider_min_mm:
        return actuator.slider_min_counts
    return desired_cts


def _command_pwm(actuator: MaxonActuator, pwm: float) -> None:
    """Set motor direction from the sign of `pwm`, then apply its magnitude."""
    if pwm < 0.0:
        actuator.set_motor_direction_forward()  # toward lower % (motor leads reversed on this VSO)
    else:
        actuator.set_motor_direction_backward()  # toward higher % (motor leads reversed on this VSO)

    actuator.set_motor_pwm(np.abs(pwm))

def go_to_position(actuator: MaxonActuator, target_perc: float, position_logger: Logger, vso=None ,ankle=None, calib_offset: float = 0.0) -> bool:
    """Command the motor to move the slider to `target_perc`.

    Returns True if it arrived within min_error before the time limit, False if it timed out.

    Direction convention assumes the encoder counts UP in the forward direction.
    """
    desired_cts = _clamp_target_cts(actuator, actuator.perc_to_cts(target_perc))
    desired_mm = actuator.cts_to_mm(desired_cts)

    start_time = time.time()
    last_time = start_time
    was_paused = False
    
    while True:
        try:
            actuator.update()
            actuator.check_coupler_drift()

            if vso is not None:
                vso.update()
            if ankle is not None:
                current_angle = SIDE * np.rad2deg(ankle.position) - calib_offset

                
                if not (-2.5 <= current_angle <= 5.0):
                        if not was_paused:
                            _command_pwm(actuator, 0.0)
                            position_logger.info(
                                f"Ankle out of range ({current_angle:.1f}°). Waiting for swing..."
                            )
                            was_paused = True
                        
                        time.sleep(DT)
                        last_time = time.time()  # Prevent PID spike when resuming
                        continue

            if was_paused:
                position_logger.info("Swing detected. Resuming slider motion.")
                was_paused = False
                start_time = time.time()
                last_time = time.time() - DT

            if np.abs(desired_mm - actuator.motor_position_mm) < actuator.min_error:
                actuator.stop()
                return True

            current_time = time.time()
            dt = current_time - last_time
            if dt <= 0.0:
                dt = 1e-6  # guard against divide-by-zero in the PID
            last_time = current_time

            if current_time - start_time > actuator.time_limit:
                position_logger.warning("Slider may be jammed - check prototype (PWM set to zero for safety).")
                actuator.stop()
                return False

            error_cts = int(desired_cts - actuator.motor_position_cts)
            pwm = actuator.pid_ctrl_position(error_cts, dt)

            if pwm == 0.0:
                actuator.stop()
                return True

            _command_pwm(actuator, pwm)
            time.sleep(DT)

        except KeyboardInterrupt:
            actuator.stop()
            position_logger.warning("KeyboardInterrupt during slider motion.")
            return False


def _adc_voltage(adc: ADS114S0x, channel_name: str) -> float:
    """Return the reading of the named ADC channel in volts, or 0.0 if unavailable."""
    data = getattr(adc, "_data", None)
    names = list(adc._channels)
    if not data or channel_name not in names:
        return 0.0
    return data[names.index(channel_name)] / 1000.0  # ADC data is in millivolts

def zero_ankle_encoder(encoder: AS5048B) -> float:
    """Average ZERO_SAMPLES readings; return the offset in degrees."""
    samples = []
    for _ in range(ZERO_SAMPLES):
        encoder.update()
        samples.append(SIDE * np.rad2deg(encoder.position))
        time.sleep(DT)
    return float(np.mean(samples))


def _input_thread(commands: "queue.Queue[str]") -> None:
    """Forward stdin lines to the main loop without blocking it."""
    while True:
        try:
            commands.put(input().strip().lower())
        except EOFError:
            commands.put("q")
            return


def build_vso() -> VSO:
    """Build the VSO with only the enabled sensors."""
    sensors = {}
    sensors["adc"] = ADS114S0x(
        offline=OFFLINE,
        tag="adc",
        spi_bus=1,
        spi_cs = 1,
        data_rate=2000,
        drdy=16,
        voltage_reference=1.65,
    )
    if ENABLE_HALL:
        sensors["hallEffect_sensor1"] = DRV5056(
            offline=OFFLINE,
            tag="hall_effect1",
            sensor_num="A1",
            t_a=23,
            supply_voltage=3.3,
        )
        sensors["hallEffect_sensor2"] = DRV5056(
            offline=OFFLINE,
            tag="hall_effect2",
            sensor_num="A1",
            t_a=23,
            supply_voltage=3.3,
        )

    sensors["ankle_encoder"] = AS5048B(
        offline=OFFLINE,
        tag="joint_encoder_ankle",
        bus=3,
        A1_adr_pin=False,
        A2_adr_pin=True,
        zero_position=0,
        enable_diagnostics=False,
    )
    return VSO[MaxonActuator, SensorBase](
        tag="variableStiffnessOrthosis", actuators={}, sensors=sensors
    )


def run(data_logger: Logger) -> None:
    """Home, zero, then run the interactive move/record loop."""
    actuator = MaxonActuator(frequency=FREQUENCY, offline=OFFLINE)
    encoder_counter = LS7366R()
    vso = build_vso()

    adc = vso.sensors.get("adc")
    hall1 = vso.sensors.get("hallEffect_sensor1")
    hall2 = vso.sensors.get("hallEffect_sensor2")
    ankle = vso.sensors.get("ankle_encoder")
    if not vso.sensors:
        LOGGER.error("No sensors enabled. Aborting.")
        return

    state = {"t": 0.0, "trial": 0, "stiff": 0.0, "battery": 0.0}
    columns = [("time_s", "t"), ("trial", "trial"), ("stiffness_meas_pct", "stiff")]
    console_fields = []
    if ankle is not None:
        state["angle"] = 0.0
        columns.append(("ankleEncoderPos", "angle"))
        console_fields.append(("angle", "angle", "+7.2f", "°"))
    if hall1 is not None:
        state["hall1"] = 0.0
        columns.append(("hallEffect_data1", "hall1"))
        console_fields.append(("hall1", "hall1", "+.4f", " V"))
    if hall2 is not None:
        state["hall2"] = 0.0
        columns.append(("hallEffect_data2", "hall2"))
        console_fields.append(("hall2", "hall2", "+.4f", "V"))
    for name, key in columns:
        data_logger.track_function(lambda k=key: state[k], name=name)

    commands: "queue.Queue[str]" = queue.Queue()
    readout_interval = FREQUENCY // READ_FREQUENCY

    gpio_handle = lgpio.gpiochip_open(RP1_GPIOCHIP)
    lgpio.gpio_claim_output(gpio_handle, MUX0_PIN, 0)  # ADG852 IN low = S2 to D (Hall VCC)
    lgpio.gpio_claim_output(gpio_handle, MUX1_PIN, 0)

    try:
        with actuator:
            # 1) Home motor reusing active actuator reference
            home_motor(actuator, encoder_counter)
            time.sleep(2)
            actuator.set_control_mode(CONTROL_MODES.POSITION)
            actuator.position_control_init()

            target = float(np.clip(float(0.05), STIFFNESS_MIN, STIFFNESS_MAX))
            arrived = go_to_position(actuator, target, LOGGER)
            actuator.update()

            with vso:
                if adc is not None:
                    adc.adc_configure_common(single_shot=True, filter_low_latency=True)
                    if hall1 is not None:
                        hall1.configure()
                        adc._channels["hall_drv5056_ain6"] = ChannelConfig(
                            name="hall_drv5056_ain6",
                            ain_pos_code=adc._ADS_P_AIN6,
                            postprocess=None,
                            units="V",
                        )
                    if hall2 is not None:
                        hall2.configure()
                        adc._channels["hall_drv5056_ain5"] = ChannelConfig(
                            name="hall_drv5056_ain5",
                            ain_pos_code=adc._ADS_P_AIN5,
                            postprocess=None,
                            units="V",
                        )
                    adc._channels["battery_voltage_monitor"] = ChannelConfig(
                        name="battery_voltage_monitor",
                        ain_pos_code=adc._ADS_P_AIN0,
                        postprocess=None,
                        units="V",
                    )
                # Zero ankle encoder at current pose
                calib_offset = 0.0
                if ankle is not None:
                    LOGGER.info("Zeroing ankle encoder. Hold the ankle unloaded and still.")
                    time.sleep(2)
                    calib_offset = zero_ankle_encoder(ankle)
                    LOGGER.info(f"Ankle zeroed. offset={calib_offset:.4f} deg")

                LOGGER.info("Ready. Type a stiffness (0-100), r, s, z, or q.")
                threading.Thread(target=_input_thread, args=(commands,), daemon=True).start()

                recording = False
                angle = None
                t_start = time.monotonic()
                loop_count = 0
                next_tick = time.monotonic()

                while True:
                    vso.update()
                    actuator.update()

                    quit_now = False
                    while not commands.empty():
                        text = commands.get()
                        if text == "q":
                            quit_now = True
                        elif text == "r":
                            state["trial"] += 1
                            recording = True
                            LOGGER.info(f"\nRecording trial {state['trial']}.")
                        elif text == "s":
                            recording = False
                            LOGGER.info("\nRecording stopped.")
                        elif text == "z":
                            if recording:
                                LOGGER.warning("\nStop recording before re-zeroing.")
                            elif ankle is not None and angle is not None:
                                calib_offset += angle
                                angle = None
                                LOGGER.info("\nAnkle re-zeroed.")
                        else:
                            try:
                                target = float(
                                    np.clip(float(text), STIFFNESS_MIN, STIFFNESS_MAX)
                                )
                            except ValueError:
                                LOGGER.warning(f"\nUnknown command: {text!r}")
                                continue
                            if recording:
                                LOGGER.warning("\nStop recording (s) before moving the slider.")
                                continue
                            arrived = go_to_position(actuator, target, LOGGER,vso,ankle,calib_offset)
                            actuator.update()
                                    
                            next_tick = time.monotonic()
                    if quit_now:
                        break

                    # Sensors update
                    if ankle is not None:
                        angle_raw = SIDE * np.rad2deg(ankle.position) - calib_offset
                        angle = (
                            angle_raw
                            if angle is None
                            else ENCODER_ALPHA * angle_raw + (1 - ENCODER_ALPHA) * angle
                        )
                        state["angle"] = angle
                    if hall1 is not None:
                        state["hall1"] = _adc_voltage(adc, "hall_drv5056_ain6")
                    if hall2 is not None:
                        state["hall2"] = _adc_voltage(adc, "hall_drv5056_ain5")
                    state["battery"] = BAT_0 + (_adc_voltage(adc, "battery_voltage_monitor") - ADC_0) * (BAT_1 - BAT_0) / (ADC_1 - ADC_0)
                    state["t"] = time.monotonic() - t_start
                    state["stiff"] = actuator.cts_to_perc(actuator.motor_position_cts)

                    if recording:
                        data_logger.update()
                        data_logger.flush_buffer()

                    if loop_count % readout_interval == 0:
                        sensor_txt = "  ".join(
                            f"{label}={state[key]:{spec}}{unit}"
                            for label, key, spec, unit in console_fields
                        )
                        line = (
                            f"  t={state['t']:6.1f}s  stiff={state['stiff']:5.1f}% batt_volts={state['battery']:5.1f}V"
                            f"  {sensor_txt}  REC trial {state['trial']}"
                        )
                        print(f"\r{line:<90}", end="", flush=True)
                    loop_count += 1

                    next_tick += DT
                    time.sleep(max(0.0, next_tick - time.monotonic()))

                LOGGER.info("\nReturning to 0% and shutting down.")
                go_to_position(actuator, 0.0, LOGGER, vso, ankle, calib_offset)

    finally:
        lgpio.gpio_free(gpio_handle, MUX0_PIN)
        lgpio.gpio_free(gpio_handle, MUX1_PIN)
        lgpio.gpiochip_close(gpio_handle)
if __name__ == "__main__":
    run(Logger(log_path="./logs", file_name="stiffness_sweep"))