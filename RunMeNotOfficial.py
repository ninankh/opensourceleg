"""Home the VSO, zero the ankle encoder, then move slider and record on demand.

Console commands (type and press Enter):
    <number>   move slider to this stiffness (0-100 %), only when not recording
    r          start recording (new trial index)
    s          stop recording
"""

import queue
import threading
import time
from datetime import datetime

import lgpio
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

# Hardware / I/O
MUX0_PIN = 35
MUX1_PIN = 39
RP1_GPIOCHIP = 0
ENABLE_HALL = True  # choose which sensors are used, logged, and printed.
OFFLINE = False
ENABLE_SYNC = False
SYNC_PI_IP = "35.3.155.109"
SYNC_PI_PORT = 5555
SYNC_TIMEOUT_S = 0.5

# ADC Measurement Calibration Factors
BAT_0 = 9.9  # V
BAT_1 = 12.6  # V
ADC_0 = 0.318  # V
ADC_1 = 3.296  # V
VOLTAGE_REFERENCE = 1.65  # V

# Timing
FREQUENCY = 200  # main loop rate (Hz)
DT = 1 / FREQUENCY  # main loop sleep, sleep while paused in a move, spacing of the zeroing samples
READ_FREQUENCY = 10  # console readout rate (Hz)
RUN_STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")  # shared by all files from this run
SETTLE_WAIT = 2  # wait period to ensure calibration steps finish
DATA_RATE = 2000  # adc data rate

# Ankle Encoder
ENCODER_LATERAL = True  # encoder on the lateral side of the ankle; False if medial
SIDE = None  # encoder sign (+1/-1), set from the leg-side prompt at startup
ENCODER_ALPHA = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)
ZERO_SAMPLES = 100  # encoder samples averaged when zeroing

# Slider Geometry
STROKE_MM = 62.5  # slider stroke (mm)
SCALE_PERC = 19972.65  # encoder counts per 1 % of slider travel
SCALE = SCALE_PERC * 100.0 / STROKE_MM  # encoder counts per mm (~31956)

# Slider Controller
MIN_POS_ERROR_MM = 0.3  # arrival window (mm)
APPROACH_GAIN = 0.4  # PWM cap per mm of remaining error (0.85 at 5 mm, 0.4 at 1 mm)
APPROACH_FLOOR = 0.3  # min PWM while closing in; ~0.15 stalls under load (logged 2026-11-18)
PWM_MINIMUM_COMMAND = 0.15  # min PWM safety
STALL_COUNTS = 200  # less motion than this (~0.006 mm) counts as stalled
STALL_WINDOW_S = 0.2  # stalled this long -> raise the PWM floor
STALL_BOOST_STEP = 0.05
STALL_BOOST_MAX = 0.3  # floor can rise to APPROACH_FLOOR + 0.3

# Homing and Back-Off After Homing
HOMING_PWM = 0.45  # home the motor

# Ankle Pause Band
ANKLE_PAUSE_MIN_DEG = -2.5  # pause the slider below this ankle angle
ANKLE_PAUSE_MAX_DEG = 5.0  # pause the slider above this ankle angle
ANKLE_HYST_DEG = 1.0  # must come this far back inside the band to resume
ANKLE_RESUME_S = 0.1  # ...and stay there this long (s)


def home_motor(actuator: MaxonActuator, encoder_counter: LS7366R) -> None:
    """Home the provided actuator using the given encoder counter."""

    actuator.set_motor_encoder(encoder_counter)
    actuator.position_control_config(scale_perc=SCALE_PERC, scale=SCALE, min_pos_error=MIN_POS_ERROR_MM)

    actuator.home(home_zero=False, homing_pwm=HOMING_PWM)
    encoder_counter.clear_counter()

    actuator.update()


def _clamp_target_cts(actuator: MaxonActuator, desired_cts: int) -> float:
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


class SyncPi:
    """Non-blocking ZMQ subscriber for the sync Pi (port of legacy ZmqSwitchPi)."""

    def __init__(self, address: str, timeout_s: float = SYNC_TIMEOUT_S) -> None:
        import zmq  # only needed when ENABLE_SYNC is True

        self._zmq = zmq
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.connect(address)
        self._timeout_s = timeout_s
        self._last_msg_time = time.monotonic()
        self._value = 0
        self._warned = False

    def update(self) -> int:
        """Drain pending messages; return the latest value, or 0 if stale."""
        while True:
            try:
                message = self._socket.recv(self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
            try:
                self._value = int(message)
            except ValueError:
                continue
            self._last_msg_time = time.monotonic()
            self._warned = False
        if time.monotonic() - self._last_msg_time > self._timeout_s:
            if not self._warned:
                LOGGER.warning("No signal from the sync Pi.")
                self._warned = True
            return 0
        return self._value

    def close(self) -> None:
        self._socket.close()
        self._context.term()
        
        
def _log_step(
    actuator: MaxonActuator,
    data_logger: "Logger | None",
    state: "dict | None",
    t_start: float,
    pwm: float,
    angle: float,
) -> None:
    """Write one control-step row to the main CSV (buffered, no flush)."""
    if data_logger is None or state is None:
        return
    state["t"] = time.monotonic() - t_start
    state["stiff"] = actuator.cts_to_perc(actuator.motor_position_cts)
    state["pwm"] = pwm
    if "angle" in state:
        state["angle"] = angle
    data_logger.update()


def go_to_position(  # noqa: C901
    actuator: MaxonActuator,
    target_perc: float,
    position_logger: Logger,
    ankle=None,
    calib_offset: float = 0.0,
    data_logger: "Logger | None" = None,
    state: "dict | None" = None,
    t_start: float = 0.0,
    sync: "SyncPi | None" = None,
) -> bool:
    """Command the motor to move the slider to `target_perc`.

    Returns True if it arrived within min_error before the time limit,
    False if it timed out. Every control step (including ankle pauses)
    is logged to the main CSV with move_id > 0.
    """
    desired_cts = _clamp_target_cts(actuator, actuator.perc_to_cts(target_perc))
    desired_mm = actuator.cts_to_mm(desired_cts)

    start_time = time.monotonic()
    was_paused = False
    in_range_since = None
    stall_ref_cts = None
    stall_t0 = start_time
    stall_boost = 0.0
    current_angle = float("nan")  # logged as NaN when no ankle encoder
    wait_count = 0  # paused control steps, throttles the waiting readout
    readout_interval = FREQUENCY // READ_FREQUENCY
    resume_lo = ANKLE_PAUSE_MIN_DEG + ANKLE_HYST_DEG
    resume_hi = ANKLE_PAUSE_MAX_DEG - ANKLE_HYST_DEG
    target_disp = actuator.cts_to_perc(desired_cts)  # clamped target (%)
    if state is not None:
        state["n_moves"] += 1
        state["move_id"] = state["n_moves"]
        state["target"] = actuator.cts_to_perc(desired_cts)  # clamped target
        for key in ("hall1", "hall2", "battery"):  # not read during moves -> NaN, not stale
            if key in state:
                state[key] = float("nan")

    try:
        while True:
            actuator.update()
            
            if sync is not None and state is not None:
                state["sync"] = sync.update()

            if ankle is not None:
                ankle.update()  # only the ankle is needed here; skip ADC/Hall reads
                current_angle = _ankle_angle_deg(ankle, calib_offset)

                if was_paused:  # tighter band to resume (hysteresis)
                    in_range = (
                        ANKLE_PAUSE_MIN_DEG + ANKLE_HYST_DEG <= current_angle <= ANKLE_PAUSE_MAX_DEG - ANKLE_HYST_DEG
                    )
                else:
                    in_range = ANKLE_PAUSE_MIN_DEG <= current_angle <= ANKLE_PAUSE_MAX_DEG

                if not in_range:
                    if not was_paused:
                        actuator.stop()
                        position_logger.info(f"Ankle out of range ({current_angle:.1f}°). " "Waiting for swing...")
                        was_paused = True
                    in_range_since = None
                    _log_step(actuator, data_logger, state, t_start, 0.0, current_angle)
                    if wait_count % readout_interval == 0:
                        print(
                            f"\r  WAITING  target={target_disp:5.1f}%"
                            f"  stiff={actuator.cts_to_perc(actuator.motor_position_cts):5.1f}%"
                            f"  angle={current_angle:+7.2f}°"
                            f"  resume band {resume_lo:+.1f}..{resume_hi:+.1f}°\033[K",
                            end="",
                            flush=True,
                        )
                    wait_count += 1
                    time.sleep(DT)
                    continue

                if was_paused:  # back in range: wait ANKLE_RESUME_S before resuming
                    if in_range_since is None:
                        in_range_since = time.monotonic()
                    if time.monotonic() - in_range_since < ANKLE_RESUME_S:
                        _log_step(actuator, data_logger, state, t_start, 0.0, current_angle)
                        if wait_count % readout_interval == 0:
                            print(
                            f"\r  WAITING  target={target_disp:5.1f}%"
                            f"  stiff={actuator.cts_to_perc(actuator.motor_position_cts):5.1f}%"
                            f"  angle={current_angle:+7.2f}°"
                            f"  resume band {resume_lo:+.1f}..{resume_hi:+.1f}°\033[K",
                            end="",
                            flush=True,
                        )
                        wait_count += 1
                        time.sleep(DT)
                        continue

            if was_paused:
                print()  # end the \r waiting line so the log starts on a new line
                wait_count = 0
                position_logger.info("Swing detected. Resuming slider motion.")
                was_paused = False
                in_range_since = None
                start_time = time.monotonic()
                stall_ref_cts = None  # restart stall detection after a pause

            if np.abs(desired_mm - actuator.motor_position_mm) < actuator.min_error:
                actuator.stop()
                return True

            current_time = time.monotonic()

            if current_time - start_time > actuator.time_limit:
                position_logger.warning("Slider may be jammed - check prototype " "(PWM set to zero for safety).")
                actuator.stop()
                return False

            error_cts = int(desired_cts - actuator.motor_position_cts)

            # Saturated P control: full PWM far away, ramp down near the target,
            # floor (+ stall boost) to beat stiction. Sign of the error sets direction.
            error_mm = abs(actuator.cts_to_mm(error_cts))
            moved = actuator.motor_position_cts - (stall_ref_cts or 0)
            if stall_ref_cts is None or abs(moved) > STALL_COUNTS:
                stall_ref_cts = actuator.motor_position_cts
                stall_t0 = current_time
                stall_boost = 0.0
            elif current_time - stall_t0 > STALL_WINDOW_S:
                stall_boost = min(stall_boost + STALL_BOOST_STEP, STALL_BOOST_MAX)
                stall_t0 = current_time
            pwm_cap = max(APPROACH_FLOOR + stall_boost, APPROACH_GAIN * error_mm)
            pwm = float(np.sign(error_cts)) * pwm_cap
            _command_pwm(actuator, pwm)

            # Signed duty actually applied (after set_motor_pwm clamping)
            pwm_applied = float(np.sign(pwm)) * actuator.speed_control.value
            _log_step(actuator, data_logger, state, t_start, pwm_applied, current_angle)

    except KeyboardInterrupt:
        actuator.stop()
        position_logger.warning("KeyboardInterrupt during slider motion.")
        return False

    finally:
        actuator.stop()
        _log_step(actuator, data_logger, state, t_start, 0.0, current_angle)
        if state is not None:
            state["move_id"] = 0
        if data_logger is not None:
            data_logger.flush_buffer()


def _adc_voltage(adc: ADS114S0x, channel_name: str) -> float:
    """Return the reading of the named ADC channel in volts, or NaN if unavailable."""
    data = getattr(adc, "_data", None)
    names = list(adc._channels)
    if not data or channel_name not in names:
        return float("nan")
    return data[names.index(channel_name)] / 1000.0  # ADC data is in millivolts


def _battery_voltage(adc_v: float) -> float:
    """Map the monitor-channel voltage to battery voltage (two-point linear cal)."""
    return BAT_0 + (adc_v - ADC_0) * (BAT_1 - BAT_0) / (ADC_1 - ADC_0)


def _wrap_deg(angle: float) -> float:
    """Wrap an angle to [-180, 180) deg."""
    return (angle + 180.0) % 360.0 - 180.0


def _ankle_angle_deg(encoder: AS5048B, calib_offset: float) -> float:
    """Ankle angle (deg, dorsiflexion +) relative to the zero pose."""
    return _wrap_deg(SIDE * np.rad2deg(encoder.position) - calib_offset)


def zero_ankle_encoder(encoder: AS5048B) -> float:
    """Average ZERO_SAMPLES readings; return the offset in degrees."""
    samples = []
    for _ in range(ZERO_SAMPLES):
        encoder.update()
        samples.append(SIDE * np.rad2deg(encoder.position))
        time.sleep(DT)
    rad = np.deg2rad(samples)
    offset = float(np.rad2deg(np.arctan2(np.mean(np.sin(rad)), np.mean(np.cos(rad)))))
    return offset


def _input_thread(commands: "queue.Queue[str]") -> None:
    """Forward stdin lines to the main loop without blocking it."""
    while True:
        try:
            commands.put(input().strip())
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
        spi_cs=1,
        data_rate=DATA_RATE,
        drdy=16,
        voltage_reference=VOLTAGE_REFERENCE,
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
        bus="/dev/i2c-1",
        A1_adr_pin=False,
        A2_adr_pin=False,
        zero_position=0,
        enable_diagnostics=False,
    )

    return VSO[MaxonActuator, SensorBase](tag="variableStiffnessOrthosis", actuators={}, sensors=sensors)


def _safe_name(text: str) -> str:
    """Make a trial name filename-safe: keep letters, digits, - and _."""
    return "".join(c if c.isalnum() or c in "-_" else "_" for c in text).strip("_")


def _wait_for_enter(prompt: str) -> None:
    """Block until the user presses Enter (Ctrl+C aborts)."""
    input(f"{prompt} Press Enter to continue (Ctrl+C to abort)...")


def _ask_side() -> str:
    """Prompt until the user enters the leg side; return 'left' or 'right'."""
    while True:
        answer = input("Leg side (l/r): ").strip().lower()
        if answer in ("l", "left"):
            return "left"
        if answer in ("r", "right"):
            return "right"
        print("Please type l or r.")


def _encoder_sign(side: str) -> int:
    """Encoder sign so dorsiflexion reads positive.

    left lateral = +1, right lateral = -1 (from the original SIDE comment);
    a medial mount mirrors the encoder, flipping the sign.
    """
    sign = 1 if side == "left" else -1
    return sign if ENCODER_LATERAL else -sign


def run(data_logger: Logger, side: str) -> None:  # noqa: C901
    """Home, zero, then run the interactive move/record loop."""
    actuator = MaxonActuator(frequency=FREQUENCY, offline=OFFLINE, pwm_minimum_command=PWM_MINIMUM_COMMAND)
    encoder_counter = LS7366R()
    vso = build_vso()

    adc = vso.sensors.get("adc")
    hall1 = vso.sensors.get("hallEffect_sensor1")
    hall2 = vso.sensors.get("hallEffect_sensor2")
    ankle = vso.sensors.get("ankle_encoder")
    
    sync = SyncPi(f"tcp://{SYNC_PI_IP}:{SYNC_PI_PORT}") if ENABLE_SYNC else None

    state = {
        "t": 0.0,
        "trial": 0,
        "move_id": 0,  # >0 during slider moves, 0 otherwise
        "n_moves": 0,  # internal counter, not logged
        "target": float("nan"),  # clamped commanded stiffness (%)
        "stiff": 0.0,
        "pwm": 0.0,  # signed applied motor duty
        "battery": 0.0,
    }
    columns = [
        ("time_s", "t"),
        ("trial", "trial"),
        ("move_id", "move_id"),
        ("stiffness_target_pct", "target"),
        ("stiffness_meas_pct", "stiff"),
        ("motor_pwm", "pwm"),
        ("battery_voltage", "battery"),
    ]
    console_fields = []
    if ankle is not None:
        state["angle"] = 0.0  # raw, logged
        state["angle_ema"] = 0.0  # smoothed, console only
        columns.append(("ankleEncoderPos", "angle"))
        console_fields.append(("angle", "angle_ema", "+7.2f", "°"))
    if hall1 is not None:
        state["hall1"] = 0.0
        columns.append(("hallEffect_data1", "hall1"))
        console_fields.append(("hall1", "hall1", "+.4f", " V"))
    if hall2 is not None:
        state["hall2"] = 0.0
        columns.append(("hallEffect_data2", "hall2"))
        console_fields.append(("hall2", "hall2", "+.4f", " V"))
    if sync is not None:
        state["sync"] = 0
        columns.insert(1, ("sync_signal", "sync"))  # 2nd column, as in the legacy CSV
        console_fields.append(("sync", "sync", "d", ""))
    for name, key in columns:
        data_logger.track_function(lambda k=key: state[k], name=name)

    data_logger.track_function(
        lambda: datetime.now().isoformat(timespec="milliseconds"),
        name="recorded_at",
    )

    commands: queue.Queue[str] = queue.Queue()
    readout_interval = FREQUENCY // READ_FREQUENCY

    try:
        gpio_handle = lgpio.gpiochip_open(RP1_GPIOCHIP)
        lgpio.gpio_claim_output(gpio_handle, MUX0_PIN, 0)  # ADG852 IN low = S2 to D (Hall VCC)
        lgpio.gpio_claim_output(gpio_handle, MUX1_PIN, 0)

        with actuator:
            _wait_for_enter("Ready to home the slider to 0%. Keep hands clear.")
            # 1) Home motor reusing active actuator reference
            home_motor(actuator, encoder_counter)
            time.sleep(SETTLE_WAIT)
            actuator.set_control_mode(CONTROL_MODES.POSITION)

            # Move to maximum stiffness before starting the session
            start_perc = actuator.slider_max_perc
            _wait_for_enter(f"Ready to move the slider to {start_perc:g} % (stiffest).")
            LOGGER.info(f"Moving slider to {start_perc:g} %.")
            if not go_to_position(actuator, start_perc, LOGGER):
                raise RuntimeError(f"Slider did not reach {start_perc:g} % at startup; aborting.")
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
                    time.sleep(SETTLE_WAIT)
                    calib_offset = zero_ankle_encoder(ankle)
                    LOGGER.info(f"Ankle zeroed. offset={calib_offset:.4f} deg")

                LOGGER.info("Ready. Type a stiffness (0.5-99.5), r, s.")
                threading.Thread(target=_input_thread, args=(commands,), daemon=True).start()

                recording = False
                awaiting_name = False
                angle_ema = None
                t_start = time.monotonic()
                loop_count = 0
                next_tick = time.monotonic()

                while True:
                    vso.update()
                    actuator.update()
                    
                    if sync is not None:
                        state["sync"] = sync.update()

                    quit_now = False
                    while not commands.empty():
                        text = commands.get()
                        if awaiting_name:  # this line is the trial name
                            awaiting_name = False
                            name = _safe_name(text) or "trial"
                            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            data_logger.set_file_name(f"{name}_{side}_{stamp}")
                            state["trial"] += 1
                            state["n_moves"] = 0  # move_id restarts at 1 in each file
                            t_start = time.monotonic()  # time_s restarts at 0 in each file
                            recording = True
                            LOGGER.info(f"Recording trial {state['trial']} -> " f"{data_logger.csv_path}")
                            continue
                        cmd = text.lower()
                        if cmd == "q":
                            quit_now = True
                        elif cmd == "r":
                            if recording:  # r while recording: close this file first
                                data_logger.close()
                                recording = False
                                LOGGER.info("Recording stopped.")
                            awaiting_name = True
                            print("\nTrial name: ", end="", flush=True)
                        elif cmd == "s":
                            if recording:
                                data_logger.close()
                                recording = False
                                LOGGER.info(f"Recording stopped. Saved " f"{data_logger.csv_path}")
                        else:
                            try:
                                target = float(cmd)
                            except ValueError:
                                LOGGER.warning(f"Unknown command: {text!r}")
                                continue
                            if not np.isfinite(target):
                                LOGGER.warning(f"Target must be finite: {text!r}")
                                continue
                            lo = actuator.slider_min_perc
                            hi = actuator.slider_max_perc
                            if not lo <= target <= hi:
                                LOGGER.warning(
                                    f"{target:g} % is outside the slider limits "
                                    f"({lo:g}-{hi:g} %); it will be clamped."
                                )
                            go_to_position(
                                actuator,
                                target,
                                LOGGER,
                                ankle,
                                calib_offset,
                                data_logger=data_logger if recording else None,
                                state=state,
                                t_start=t_start,
                                sync=sync,
                            )
                            actuator.update()
                            next_tick = time.monotonic()
                    if quit_now:
                        break

                    # Sensors update
                    if ankle is not None:
                        angle_raw = _ankle_angle_deg(ankle, calib_offset)
                        angle_ema = (
                            angle_raw
                            if angle_ema is None
                            else ENCODER_ALPHA * angle_raw + (1 - ENCODER_ALPHA) * angle_ema
                        )
                        state["angle"] = angle_raw
                        state["angle_ema"] = angle_ema
                    if hall1 is not None:
                        state["hall1"] = _adc_voltage(adc, "hall_drv5056_ain6")
                    if hall2 is not None:
                        state["hall2"] = _adc_voltage(adc, "hall_drv5056_ain5")
                    state["battery"] = _battery_voltage(_adc_voltage(adc, "battery_voltage_monitor"))
                    state["t"] = time.monotonic() - t_start
                    state["stiff"] = actuator.cts_to_perc(actuator.motor_position_cts)

                    if recording:
                        data_logger.update()
                        data_logger.flush_buffer()

                    if not awaiting_name and loop_count % readout_interval == 0:
                        sensor_txt = "  ".join(
                            f"{label}={state[key]:{spec}}{unit}" for label, key, spec, unit in console_fields
                        )
                        rec_txt = f"REC trial {state['trial']}" if recording else "idle"
                        line = (
                            f"  t={state['t']:6.1f}s  target={state['target']:5.1f}%"
                            f"  stiff={state['stiff']:5.1f}%"
                            f"  batt_volts={state['battery']:5.1f}V"
                            f"  {sensor_txt}  {rec_txt}"
                        )
                        print(f"\r{line}\033[K", end="", flush=True)
                    loop_count += 1

                    next_tick += DT
                    time.sleep(max(0.0, next_tick - time.monotonic()))

                LOGGER.info("Quitting; slider left at its current position.")

    finally:
        data_logger.close()  # flush/close any open trial CSV (safe if none is open)
        if sync is not None:
            sync.close()
        lgpio.gpio_free(gpio_handle, MUX0_PIN)
        lgpio.gpio_free(gpio_handle, MUX1_PIN)
        lgpio.gpiochip_close(gpio_handle)


if __name__ == "__main__":
    side = _ask_side()
    SIDE = _encoder_sign(side)  # module-level: rebinds the global used by all functions
    print(
        f"Side: {side}, encoder {'lateral' if ENCODER_LATERAL else 'medial'}, "
        f"SIDE = {SIDE:+d}. Pause band {ANKLE_PAUSE_MIN_DEG}..{ANKLE_PAUSE_MAX_DEG} deg "
        "(dorsiflexion positive)."
    )
    run(Logger(log_path="./logs", file_name=f"stiffness_sweep_{side}_{RUN_STAMP}"), side)
