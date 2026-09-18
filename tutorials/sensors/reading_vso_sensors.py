import time

import numpy as np

from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import Logger
from opensourceleg.robots.vso import VSO
from opensourceleg.sensors.adc import ADS114S0x, ChannelConfig
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.sensors.hall import DRV5056
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop

FREQUENCY = 200  # control/read loop rate (Hz)
READ_FREQUENCY = 10  # console readout rate (Hz)
OFFLINE = False
SIDE = -1  # 1: left-leg lateral encoder (-1 if medial); -1: right-leg lateral (1 if medial)
ENCODER_ALPHA = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)


def _hall_voltage(adc: ADS114S0x, channel_index: int = 0) -> float:
    """Return an ADC channel reading in volts, or 0.0 if unavailable.

    Args:
        adc: The ADS114S0x instance whose latest data to read.
        channel_index: Index into adc.data for this Hall channel. Defaults to 0.

    Returns:
        float: Hall channel voltage in volts (adc.data is in millivolts).
    """
    data = adc.data
    if not data or channel_index >= len(data):
        return 0.0
    return data[channel_index] / 1000.0


def initialize_sensors() -> dict:
    need_adc = True
    need_hall = True
    need_ankleEncoder = True
    need_encoderCounter = True

    # Guard: Hall requires ADC
    if need_hall and not need_adc:
        LOGGER.warning("Hall effect requires ADC. Disabling Hall.")
        need_hall = False

    sensors = {}

    if need_adc:
        sensors["adc"] = ADS114S0x(
            offline=OFFLINE, tag="adc", spi_bus=1, data_rate=2000, drdy=16, voltage_reference=1.65
        )
    if need_hall:
        sensors["hallEffect_sensor"] = DRV5056(
            offline=OFFLINE, tag="hall_effect", sensor_num="A1", t_a=23, supply_voltage=3.3
        )
    if need_ankleEncoder:
        sensors["ankle_encoder"] = AS5048B(
            offline=OFFLINE,
            tag="joint_encoder_ankle",
            bus="/dev/i2c-3",
            A1_adr_pin=False,
            A2_adr_pin=True,
            zero_position=0,
            enable_diagnostics=False,
        )
    if need_encoderCounter:
        sensors["encoder_counter"] = LS7366R()

    return sensors


def read_sensors(data_logger: Logger) -> None:
    sensors = initialize_sensors()

    vso = VSO[MaxonActuator, SensorBase](
        tag="variableStiffnessOrthosis",
        actuators={},
        sensors=sensors,
    )
    LOGGER.info("Finished setting up VSO.")

    # setup
    adc = vso.sensors.get("adc")
    if adc is not None:
        adc.adc_configure_common(single_shot=True, filter_low_latency=True)

    hall = vso.sensors.get("hallEffect_sensor")
    if hall is not None and adc is not None:
        hall.configure()
        adc._channels["hall_drv5056_ain3"] = ChannelConfig(
            name="hall_drv5056_ain3",
            ain_pos_code=adc._ADS_P_AIN3,
            postprocess=None,
            units="mV",
        )

    # Ankle encoder calibration
    calib_offset = 0.0
    ankle_sensor = vso.sensors.get("ankle_encoder")
    if ankle_sensor is not None:
        LOGGER.info("Capturing unloaded equilibrium angle. Waiting for ankle encoder warmup.")
        time.sleep(2)
        ankle_sensor.update()
        calib_offset = SIDE * np.rad2deg(ankle_sensor.position)
        LOGGER.info(f"Ankle encoder offset calibrated. calib_offset={calib_offset:.4f} deg")
    else:
        LOGGER.warning("No ankle encoder found. Angle will not be captured.")

    encoder_counter = vso.sensors.get("encoder_counter")
    if encoder_counter is not None:
        encoder_counter.clear_counter()
        data_logger.track_function(lambda: encoder_counter.encoder_count, name="encoderCounter_count")

    # Tracked values
    angle = 0.0

    if ankle_sensor is not None:
        data_logger.track_function(lambda: angle, name="ankleEncoderPos_deg")
    if adc is not None:
        data_logger.track_function(lambda: _hall_voltage(adc, channel_index=0), name="hallEffect_V")

    readout_interval = FREQUENCY // READ_FREQUENCY

    with vso:
        loop = SoftRealtimeLoop(dt=1 / FREQUENCY)
        t_start = time.monotonic()

        for loop_count, _ in enumerate(loop):
            vso.update()

            if ankle_sensor is not None:
                angle_raw = SIDE * np.rad2deg(ankle_sensor.position) - calib_offset
                angle = angle_raw if loop_count == 0 else ENCODER_ALPHA * angle_raw + (1 - ENCODER_ALPHA) * angle

            data_logger.update()
            data_logger.flush_buffer()

            if loop_count % readout_interval == 0:
                elapsed = time.monotonic() - t_start
                hall_v = _hall_voltage(adc, channel_index=0) if adc is not None else 0.0
                print(
                    f"\r  t={elapsed:6.1f}s"
                    + (f"  angle={angle:+7.2f}°" if ankle_sensor is not None else "")
                    + (f"  hall={hall_v:+.4f} V" if adc is not None else ""),
                    end="",
                    flush=True,
                )


if __name__ == "__main__":
    data_logger = Logger(
        log_path="./logs",
        file_name="read_sensor",
    )
    read_sensors(data_logger)
