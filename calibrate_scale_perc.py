"""Measure SCALE_PERC (encoder counts per 1 % of slider travel) from a homing sweep.

Sequence: home False (0 % stop) -> clear counter -> home True (100 % stop)
-> clear counter -> home False. The counts of that last run are the full
stroke; SCALE_PERC = |counts| / 100. Appended to scale_perc.csv, which
RunMeNotOfficial.py reads at startup (last row wins).

Bench-test only, with the orthosis off the leg: homing drives into both
hard stops at HOMING_PWM and the library's home() has no time limit.
"""

import csv
import time
from datetime import datetime
from pathlib import Path

from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.sensors.encoderCounter import LS7366R

# Keep these in sync with RunMeNotOfficial.py
FREQUENCY = 200  # Hz
PWM_MINIMUM_COMMAND = 0.15
HOMING_PWM = 0.45
STROKE_MM = 62.5  # slider stroke (mm)
SCALE_PERC_DEFAULT = 19972.65  # previous hard-coded value, for comparison only

SCALE_PERC_FILE = Path(__file__).with_name("scale_perc.csv")
SETTLE_WAIT = 1.0  # s, pause after each homing run
SCALE_DEFAULT = SCALE_PERC_DEFAULT * 100.0 / STROKE_MM  # counts per mm (~31956)

# Open-loop break-away off a hard stop before homing the other way
BREAKAWAY_PWM = 0.85  # = MaxonActuator pwm_maximum_command, so it is not clipped
BREAKAWAY_MM = 0.1  # stop once moved this far
BREAKAWAY_TIMEOUT_S = 0.5  # abort if it has not moved BREAKAWAY_MM by then
MAX_LEG_MISMATCH = 0.02  # True-run and last False-run strokes must agree within 2 %
MAX_DEFAULT_DEVIATION = 0.20  # refuse results more than 20 % from the default


def _break_away(actuator: MaxonActuator, encoder_counter: LS7366R, toward_true: bool) -> None:
    """Open loop at BREAKAWAY_PWM until the slider moves BREAKAWAY_MM off the stop."""
    target_cts = BREAKAWAY_MM * SCALE_DEFAULT
    start_cts = encoder_counter.count
    if toward_true:
        actuator.set_motor_direction_backward()  # same direction as home(home_zero=True)
    else:
        actuator.set_motor_direction_forward()  # same direction as home(home_zero=False)
    t0 = time.monotonic()
    try:
        actuator.set_motor_pwm(BREAKAWAY_PWM)
        while abs(encoder_counter.count - start_cts) < target_cts:
            if time.monotonic() - t0 > BREAKAWAY_TIMEOUT_S:
                raise RuntimeError(
                    f"Slider did not move {BREAKAWAY_MM} mm off the stop within "
                    f"{BREAKAWAY_TIMEOUT_S} s at PWM {BREAKAWAY_PWM}. Aborting."
                )
            time.sleep(0.001)
    finally:
        actuator.stop()
    moved_mm = abs(encoder_counter.count - start_cts) / SCALE_DEFAULT
    LOGGER.info(f"Broke away from the stop ({moved_mm:.2f} mm).")


def _home(actuator: MaxonActuator, encoder_counter: LS7366R, home_zero: bool) -> int:
    """Home to one hard stop and return the encoder count when it stalls."""
    actuator.home(home_zero=home_zero, homing_pwm=HOMING_PWM)
    actuator.stop()
    time.sleep(SETTLE_WAIT)
    return int(encoder_counter.count)


def main() -> None:
    actuator = MaxonActuator(frequency=FREQUENCY, pwm_minimum_command=PWM_MINIMUM_COMMAND)
    encoder_counter = LS7366R()

    with actuator:
        try:
            actuator.set_motor_encoder(encoder_counter)
            # update() needs scale; values only affect logging during homing
            actuator.position_control_config(scale_perc=SCALE_PERC_DEFAULT, scale=SCALE_DEFAULT)
            input(
                "Calibration homes the slider 0% -> 100% -> 0%. Orthosis OFF the leg, "
                "hands clear. Press Enter to start (Ctrl+C to abort)..."
            )

            LOGGER.info("1/3: homing False (0 % stop).")
            _home(actuator, encoder_counter, home_zero=False)
            encoder_counter.clear_counter()

            LOGGER.info("2/3: breaking away, then homing True (100 % stop).")
            _break_away(actuator, encoder_counter, toward_true=True)
            counts_true = _home(actuator, encoder_counter, home_zero=True)
            encoder_counter.clear_counter()

            LOGGER.info("3/3: breaking away, then homing False (0 % stop), measuring stroke.")
            _break_away(actuator, encoder_counter, toward_true=False)
            counts_false = _home(actuator, encoder_counter, home_zero=False)
        finally:
            actuator.stop()

    stroke_true = abs(counts_true)
    stroke_false = abs(counts_false)
    scale_perc = stroke_false / 100.0
    mismatch = abs(stroke_true - stroke_false) / max(stroke_false, 1)
    deviation = abs(scale_perc - SCALE_PERC_DEFAULT) / SCALE_PERC_DEFAULT

    print(f"\nTrue run stroke:  {counts_true:+d} counts")
    print(f"False run stroke: {counts_false:+d} counts")
    print(f"SCALE_PERC = {scale_perc:.2f} counts/%  (default {SCALE_PERC_DEFAULT}, {deviation:+.1%} off)")
    print(f"True vs False stroke mismatch: {mismatch:.2%}")

    if mismatch > MAX_LEG_MISMATCH:
        LOGGER.error(
            f"Strokes disagree by {mismatch:.1%} (> {MAX_LEG_MISMATCH:.0%}). "
            "Possible early stall or encoder slip. Not saved; rerun."
        )
        return
    if deviation > MAX_DEFAULT_DEVIATION:
        LOGGER.error(
            f"SCALE_PERC is {deviation:.0%} from the default. Check the encoder and "
            "coupler. Not saved."
        )
        return

    is_new = not SCALE_PERC_FILE.exists()
    with open(SCALE_PERC_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(["recorded_at", "counts_home_true", "counts_home_false", "scale_perc"])
        writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                counts_true,
                counts_false,
                f"{scale_perc:.4f}",
            ]
        )
    LOGGER.info(f"Saved SCALE_PERC = {scale_perc:.2f} to {SCALE_PERC_FILE}")


if __name__ == "__main__":
    main()