import json
import select
import sys
import termios
import time
import tty
from collections import deque
from pathlib import Path
from typing import Any, cast

import numpy as np
from calibration import VSOCalibration
from sliderPosition import sliderPosition

from opensourceleg.logging import LOGGER
from opensourceleg.robots.vso import VSO
from opensourceleg.sensors.adc import ChannelConfig

DEFAULT_CALIB_OFFSET_PATH = Path("vso_calib_offset.json")
DEFAULT_HALL_THRESHOLD_PATH = Path("hall_switch_thresholds.json")


class VSOInitialization:
    """
    Orchestrates VSO startup sequence:
        1. Optional stroke calibration (0→100→0) to compute and save scale_perc.
        2. Encoder homing — drive to soft stop, zero encoder, move to 100%.
        3. Ankle encoder offset calibration — capture unloaded equilibrium angle
           at 100% stiffness and save calib_offset to file.

    Args:
        vso: The VSO instance to initialize.
        calibration_path: Path to the stroke calibration JSON file (scale_perc).
        calib_offset_path: Path to save the ankle encoder offset JSON file.
        homing_pwm: PWM value used for homing and calibration moves.
        sample_rate: Time in seconds between position samples during homing.
        position_threshold: Max position delta (±) to consider motor stopped.

    Example:
        init = VSOInitialization(vso=my_vso)
        init.run(run_calibration=False)  # normal power cycle
        init.run(run_calibration=True)   # after reassembly
    """

    def __init__(
        self,
        vso: VSO,
        calibration_path: Path = Path("vso_calibration.json"),
        calib_offset_path: Path = DEFAULT_CALIB_OFFSET_PATH,
        hall_threshold_path: Path = DEFAULT_HALL_THRESHOLD_PATH,
        homing_pwm: float = 0.25,
        sample_rate: float = 0.05,
        position_threshold: int = 100,
        side: int = 1,  # 1 for left leg lateral encoder (-1 if medial), -1 for right leg lateral encoder (1 if medial)
        bat_div_gain: float = 2.0,
    ) -> None:
        self.vso = vso
        self.calibration_path = calibration_path
        self.calib_offset_path = calib_offset_path
        self.hall_threshold_path = Path(hall_threshold_path)
        self.homing_pwm = homing_pwm
        self.sample_rate = sample_rate
        self.position_threshold = position_threshold
        self.side = side
        self.bat_div_gain = bat_div_gain
        self.calib_offset = 0.0

        LOGGER.info("VSO Initialization instance created.")

    def run(self, run_calibration: bool = False, run_hall_calibration: bool = False) -> None:
        """
        Run the full VSO initialization sequence.

        Args:
            run_calibration: If True, runs stroke calibration to compute scale_perc before homing.
                Use after disassembly or first-time setup.
            run_hall_calibration: If True, runs interactive hall switch threshold calibration
                after the ankle encoder offset is captured.  Saves results to hall_threshold_path.
        """
        actuator = self.vso.actuators["ankle"] if self.vso.actuators.get("ankle", None) is not None else None

        if self.vso.sensors.get("motor_encoder", None) is not None:
            encoder_counter = self.vso.sensors["motor_encoder"]
        else:
            encoder_counter = None

        if self.vso.sensors.get("ankle_encoder", None) is not None:
            ankle_sensor = self.vso.sensors["ankle_encoder"]
        else:
            ankle_sensor = None

        if self.vso.sensors.get("adc", None) is not None:
            adc = self.vso.sensors["adc"]
            adc.adc_configure_common(single_shot=True, filter_low_latency=True)
            # readback = adc.read_single_register(adc._REG_ADDR_DATARATE)
            # print(f"DATARATE register: 0x{readback:02X}  (expected 0x3C)")

            if self.vso.sensors.get("hallEffect_1", None) is not None:
                hall1 = self.vso.sensors.get("hallEffect_1")
                hall1.configure()
                adc._channels["hall_drv5056_ain3"] = ChannelConfig(
                    name="hall_drv5056_ain3", ain_pos_code=adc._ADS_P_AIN3, postprocess=None, units="V"
                )

            if self.vso.sensors.get("hallEffect_2", None) is not None:
                hall2 = self.vso.sensors.get("hallEffect_2")
                hall2.configure()
                adc._channels["hall_drv5056_ain4"] = ChannelConfig(
                    name="hall_drv5056_ain4", ain_pos_code=adc._ADS_P_AIN4, postprocess=None, units="V"
                )

            battery_monitor = self.vso.sensors.get("battery_monitor")
            if battery_monitor is not None:
                adc._channels["battery"] = ChannelConfig(
                    name="battery",
                    ain_pos_code=adc._ADS_P_AIN0,
                    postprocess=lambda v: self.battery_postprocess(v, self.bat_div_gain),
                    units="V",
                )

        else:
            adc = None

        time.sleep(0.05)

        if actuator is not None and encoder_counter is not None:
            calibration = VSOCalibration(
                vso=self.vso,
                actuator=actuator,
                encoder=encoder_counter,
                calibration_path=self.calibration_path,
            )

            # Step 1: Optional stroke calibration
            if run_calibration:
                LOGGER.info("Running stroke calibration.")
                scale_perc = calibration.run(
                    homing_pwm=self.homing_pwm,
                    sample_rate=self.sample_rate,
                    position_threshold=self.position_threshold,
                )
                LOGGER.info(f"Stroke calibration complete. scale_perc={scale_perc:.2f}")
            else:
                LOGGER.info("Skipping stroke calibration. Assuming calibration file exists.")

            # Step 2: Encoder homing — zero at soft stop%
            LOGGER.info("Homing to soft stop and zeroing encoder.")
            self.vso.home(
                homing_pwm=self.homing_pwm,
                sample_rate=self.sample_rate,
                position_threshold=self.position_threshold,
                home_zero=True,
            )

            encoder_counter.clearCounter()
            time.sleep(1.5)  # Ensure encoder clear is seen before moving
            LOGGER.info("Encoder zeroed.")

            LOGGER.info("Moving spring-support to stiffest position (100%).")
            scale_perc = calibration.load()

            actuator.position_control_init()
            actuator.position_control_config(scale_perc=scale_perc)

            sliderPosition.slider_position(actuator, desired_position_perc=99.5)

        if ankle_sensor is not None:
            # Step 3: Ankle encoder offset calibration at 100% stiffness
            LOGGER.info("Capturing unloaded equilibrium angle. Waiting for ankle encoder warmup.")
            time.sleep(2)  # Warmup period for ankle encoder to stabilize
            ankle_sensor.update()
            self.calib_offset = self.side * np.rad2deg(ankle_sensor.position)
            self._save_calib_offset()
            LOGGER.info(f"Ankle encoder offset calibrated. calib_offset={self.calib_offset:.4f} deg")
        else:
            LOGGER.info("No ankle encoder. Could not capture unloaded equilibrium angle.")

        # Step 4: Optional hall switch threshold calibration
        if run_hall_calibration:
            LOGGER.info("Running hall switch calibration.")
            self.calibrate_hall_switches()
            LOGGER.info("Hall switch calibration complete.")
        else:
            LOGGER.info("Skipping hall switch calibration. Assuming threshold file exists.")

        LOGGER.info("VSO initialization complete.")

    def run_manual_motor_calibration(self) -> float:
        """
        Manual stroke calibration — no motor movement.

        Prompts the user to manually rotate the motor shaft to each end stop,
        pressing Enter at each position. Uses encoder counts between the two
        positions to compute scale_perc and saves it to the calibration file.

        The encoder is zeroed at the 0% (soft stop) end stop so that subsequent
        encoder reads directly map to position percentage.

        Returns:
            scale_perc: Encoder counts per 1% of full lead-screw travel.

        Raises:
            RuntimeError: If the motor encoder is not present in VSO sensors.
        """
        encoder_counter = self.vso.sensors.get("motor_encoder")
        if encoder_counter is None:
            raise RuntimeError("Motor encoder not found in VSO sensors.")

        print()
        print("=" * 60)
        print("  Manual Motor Stroke Calibration")
        print("=" * 60)
        print("  Manually rotate the motor shaft to the 0% end stop (soft stop).")
        input("  Press Enter when at the 0% end stop... ")

        encoder_counter.clearCounter()
        LOGGER.info("Encoder zeroed at 0% end stop.")
        print("  Encoder zeroed.")

        print()
        print("  Manually rotate the motor shaft to the 100% end stop (hard stop).")
        input("  Press Enter when at the 100% end stop... ")

        encoder_counts = cast(int, encoder_counter.readCounter())
        scale_perc = abs(encoder_counts) / 100.0
        LOGGER.info(f"100% end stop recorded. Encoder counts: {encoder_counts}. " f"scale_perc: {scale_perc:.2f}")

        calibration = VSOCalibration(
            vso=self.vso,
            actuator=self.vso.actuators.get("ankle"),
            encoder=encoder_counter,
            calibration_path=self.calibration_path,
        )
        calibration.scale_perc = scale_perc

        ankle_actuator = self.vso.actuators.get("ankle")
        if ankle_actuator is None:
            raise RuntimeError("Ankle actuator not found in VSO.")

        ankle_actuator.position_control_init()
        ankle_actuator.position_control_config(scale_perc=scale_perc)

        calibration._save()

        print(f"  Calibration complete. scale_perc = {scale_perc:.2f}")
        print("=" * 60)
        print()
        return scale_perc

    def battery_postprocess(self, volts_at_adc: float, bat_div_gain: float) -> float:
        """Convert ADC-pin voltage to battery voltage (undo divider)."""
        return volts_at_adc * bat_div_gain

    def _save_calib_offset(self) -> None:
        """
        Save the ankle encoder calibration offset to file.

        Reads self.calib_offset (degrees) and writes it to self.calib_offset_path as JSON.
        """
        with open(self.calib_offset_path, "w") as f:
            json.dump({"calib_offset": self.calib_offset}, f, indent=2)
        LOGGER.info(f"calib_offset saved to {self.calib_offset_path}")

    def load_calib_offset(self) -> None:
        """
        Load the ankle encoder calibration offset from file.

        Sets self.calib_offset (degrees) from the JSON file at self.calib_offset_path.

        Raises:
            FileNotFoundError: If no calib_offset file exists at the specified path.
        """
        if not self.calib_offset_path.exists():
            raise FileNotFoundError(
                f"No calib_offset file found at {self.calib_offset_path}. " "Run VSOInitialization.run() first."
            )
        with open(self.calib_offset_path) as f:
            data = json.load(f)
        self.calib_offset = data["calib_offset"]
        LOGGER.info(f"Loaded calib_offset: {self.calib_offset:.4f} from {self.calib_offset_path}")

    def calibrate_hall_switches(
        self,
        frequency: int = 200,
        n_std_hall: float = 2.0,
        n_std_angle: float = 2.0,
        min_margin_v: float = 0.02,
        min_margin_deg: float = 1.0,
        neg_exclusion_pct: float = 90.0,
    ) -> dict[str, dict[str, "Any"]]:
        """
        Interactive calibration of hall-effect switch thresholds.

        Runs a sensor-read loop and records hall1, hall2, ankle angle, and
        their sample-to-sample derivatives at each labelled event. Switch
        windows are computed as mean ± n_std*sigma, then clipped using labelled
        non-switch samples so the windows actively exclude non-switch territory.

        Controls during the loop:
            d      — dorsiflexion switch just fired (windowed peak detection)
            p      — plantarflexion switch just fired (windowed peak detection)
            n      — not a switch right now (records current values directly)
            ENTER  — finish and save thresholds

        Call this after init.run() so that self.calib_offset is already set.

        Args:
            frequency:          Sensor-read rate in Hz (default 200).
            n_std_hall:         Half-width of hall voltage windows in sigma (default 2.0).
            n_std_angle:        Half-width of angle window in sigma (default 2.0).
            min_margin_v:       Minimum half-width for hall windows in V (default 0.02).
            min_margin_deg:     Minimum half-width for angle window in degrees (default 1.0).
            neg_exclusion_pct:  Percentile of non-switch intrusions to exclude when
                                clipping windows (default 90.0 → exclude 90 % of
                                intruding non-switch samples on each side).

        Returns:
            Threshold dict (same structure written to JSON).

        Raises:
            RuntimeError: If the ADC sensor is not present.
            ValueError:   If fewer than 1 event of either switch type was captured.
        """
        adc = self.vso.sensors.get("adc")
        ankle_sensor = self.vso.sensors.get("ankle_encoder")

        if adc is None:
            raise RuntimeError("ADC sensor not found in VSO — cannot calibrate hall switches.")

        dorsi_events = []  # peak sample at each dorsi switch
        plantar_events = []  # peak sample at each plantar switch
        no_switch_events = []  # current sample each time user marks a non-switch moment

        dt = 1.0 / frequency
        encoder_alpha = 0.15  # EMA smoothing factor (~5 Hz cutoff at 200 Hz)

        # Window sizing: 0.4 s pre-keypress lookback, 0.2 s post-keypress lookahead
        pre_samples = int(0.4 * frequency)
        post_samples = int(0.2 * frequency)

        angle_filt = 0.0
        angle_last = 0.0
        hall1_last = 0.0
        hall2_last = 0.0

        rolling_buf: deque[dict[str, float]] = deque(maxlen=pre_samples)  # continuous pre-window
        pending_key = None  # 'd' or 'p' while collecting post-window
        post_buf = []
        post_remain = 0

        print()
        print("=" * 60)
        print("  Hall Switch Calibration")
        print("=" * 60)
        print("  Walk the exoskeleton through dorsi/plantarflexion cycles.")
        print("  Press a key near the time each mechanical switch fires —")
        print("  the peak hall derivative in the surrounding window is used.")
        print("  Also press 'n' during clearly non-switch moments to help")
        print("  tighten the windows against false positives.")
        print()
        print("    d     = dorsiflexion switch")
        print("    p     = plantarflexion switch")
        print("    n     = not a switch (negative example)")
        print("    ENTER = done — compute and save thresholds")
        print("=" * 60)
        print()

        def _read_sensors() -> tuple[float, float, float]:
            """Read hall voltages and ankle angle from sensors."""
            self.vso.update()
            data = getattr(adc, "_data", [0.0, 0.0])
            h1 = data[0] / 1000.0 if len(data) > 0 else 0.0
            h2 = data[1] / 1000.0 if len(data) > 1 else 0.0
            if ankle_sensor is not None:
                ankle_sensor.update()
                ang = self.side * np.rad2deg(ankle_sensor.position) - self.calib_offset
            else:
                ang = 0.0
            return h1, h2, ang

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        t_loop_start = time.monotonic()

        try:
            tty.setcbreak(fd)

            while True:
                t_iter = time.monotonic()

                hall1, hall2, angle_raw = _read_sensors()
                angle_filt = encoder_alpha * angle_raw + (1 - encoder_alpha) * angle_filt
                hall1_dot = hall1 - hall1_last
                hall2_dot = hall2 - hall2_last
                angle_dot = angle_filt - angle_last
                angle = angle_filt

                sample = {
                    "hall1": hall1,
                    "hall2": hall2,
                    "hall1_dot": hall1_dot,
                    "hall2_dot": hall2_dot,
                    "angle": angle,
                    "angle_dot": angle_dot,
                }

                # --- Post-window collection for switch events ---
                if post_remain > 0:
                    post_buf.append(sample)
                    post_remain -= 1

                    if post_remain == 0:
                        # Find the sample in the combined window with the largest
                        # total hall derivative — that is when the switch actually fired.
                        window = list(rolling_buf) + post_buf
                        peak = max(
                            window,
                            key=lambda s: abs(s["hall1_dot"]) + abs(s["hall2_dot"]),
                        )
                        if pending_key == "d":
                            dorsi_events.append(peak)
                            label = f"DORSI  #{len(dorsi_events):02d}"
                        else:
                            plantar_events.append(peak)
                            label = f"PLANTAR #{len(plantar_events):02d}"
                        print(
                            f"\n  [{label}]"
                            f"  hall1={peak['hall1']:+.4f} V  hall2={peak['hall2']:+.4f} V"
                            f"  angle={peak['angle']:+7.2f}deg"
                            f"  d(h1)={peak['hall1_dot']:+.5f}  d(h2)={peak['hall2_dot']:+.5f}"
                        )
                        pending_key = None
                        post_buf = []

                # Always push to rolling buffer (used as pre-window for future keypresses)
                rolling_buf.append(sample)

                # --- Keypress check ---
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)

                    if key in ("\n", "\r"):
                        if post_remain > 0:
                            print("\n  (Discarding incomplete window — press ENTER again to finish.)")
                        else:
                            print("\n\nFinishing calibration...")
                            break

                    elif key.lower() in ("d", "p") and pending_key is None:
                        pending_key = key.lower()
                        post_remain = post_samples
                        post_buf = []

                    elif key.lower() == "n" and pending_key is None:
                        # Record current values directly — no windowing, we want
                        # the "steady" non-switch reading, not any derivative peak.
                        no_switch_events.append(sample)
                        print(
                            f"\n  [NO-SWITCH #{len(no_switch_events):02d}]"
                            f"  hall1={hall1:+.4f} V  hall2={hall2:+.4f} V"
                            f"  angle={angle:+7.2f}deg"
                        )

                else:
                    elapsed = time.monotonic() - t_loop_start
                    if post_remain > 0:
                        status = f"collecting +{post_samples - post_remain}/{post_samples}"
                    else:
                        status = f"d={len(dorsi_events)} p={len(plantar_events)}" f" n={len(no_switch_events)}"
                    print(
                        f"\r  t={elapsed:6.1f}s"
                        f"  angle={angle:+7.2f}deg"
                        f"  h1={hall1:+.4f} V  h2={hall2:+.4f} V"
                        f"  d(h1)={hall1_dot:+.5f}"
                        f"  [{status}]   ",
                        end="",
                        flush=True,
                    )

                angle_last = angle_filt
                hall1_last = hall1
                hall2_last = hall2

                remaining = dt - (time.monotonic() - t_iter)
                if remaining > 0:
                    time.sleep(remaining)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if not dorsi_events or not plantar_events:
            raise ValueError(
                f"Insufficient calibration data — dorsi={len(dorsi_events)} events, "
                f"plantar={len(plantar_events)} events. Need at least 1 of each."
            )

        # ------------------------------------------------------------------ #
        # Threshold computation                                                #
        # ------------------------------------------------------------------ #

        def _bounds_v(values: list[float]) -> tuple[float, float]:
            """mean ± n_std_hall*sigma, floored at min_margin_v half-width."""
            a = np.array(values)
            half = max(n_std_hall * float(a.std()), min_margin_v)
            return float(a.mean()) - half, float(a.mean()) + half

        def _bounds_deg(values: list[float]) -> tuple[float, float]:
            """mean ± n_std_angle*sigma, floored at min_margin_deg half-width."""
            a = np.array(values)
            half = max(n_std_angle * float(a.std()), min_margin_deg)
            return float(a.mean()) - half, float(a.mean()) + half

        def _dot_upper(values: list[float]) -> float:
            """mean(|derivative|) + n_std_hall*sigma, floored at min_margin_v."""
            a = np.abs(values)
            return float(a.mean()) + max(n_std_hall * float(a.std()), min_margin_v)

        def _clip_with_negatives(
            lo: float, hi: float, switch_mean: float, neg_values: list[float], is_degrees: bool = False
        ) -> tuple[float, float, int, int]:
            """
            Tighten [lo, hi] so that neg_exclusion_pct % of non-switch samples
            that intrude from each side are pushed outside the window.

            Non-switch samples below switch_mean can push the lower bound up;
            non-switch samples above switch_mean can push the upper bound down.
            If clipping makes the window degenerate (lo >= hi), the original
            bounds are kept and a warning is printed.
            """
            if not neg_values:
                return lo, hi, 0, 0

            a = np.array(neg_values)
            unit = "deg" if is_degrees else " V"
            orig_lo, orig_hi = lo, hi

            below = a[(a >= orig_lo) & (a < switch_mean)]
            above = a[(a > switch_mean) & (a <= orig_hi)]

            n_below = len(below)
            n_above = len(above)

            if n_below > 0:
                # Raise lower bound to the neg_exclusion_pct-th percentile of intruders
                clip_lo = float(np.percentile(below, neg_exclusion_pct))
                lo = max(lo, clip_lo)

            if n_above > 0:
                # Lower upper bound to the (100 - neg_exclusion_pct)-th percentile
                clip_hi = float(np.percentile(above, 100.0 - neg_exclusion_pct))
                hi = min(hi, clip_hi)

            if lo >= hi:
                LOGGER.warning(
                    f"Negative-example clipping produced a degenerate window "
                    f"({lo:.4f}{unit} >= {hi:.4f}{unit}). Reverting to original bounds."
                )
                return orig_lo, orig_hi, n_below, n_above

            return lo, hi, n_below, n_above

        # --- Extract switch event values ---
        d_h1 = [e["hall1"] for e in dorsi_events]
        d_h2 = [e["hall2"] for e in dorsi_events]
        d_h1_dot = [e["hall1_dot"] for e in dorsi_events]
        d_ang = [e["angle"] for e in dorsi_events]
        d_ang_dot = [e["angle_dot"] for e in dorsi_events]

        p_h1 = [e["hall1"] for e in plantar_events]
        p_h2 = [e["hall2"] for e in plantar_events]
        p_ang = [e["angle"] for e in plantar_events]
        p_ang_dot = [e["angle_dot"] for e in plantar_events]

        # --- Non-switch values (same dimensions) ---
        ns_h1 = [e["hall1"] for e in no_switch_events]
        ns_h2 = [e["hall2"] for e in no_switch_events]
        ns_ang = [e["angle"] for e in no_switch_events]

        # --- Initial windows from switch events ---
        d_h1_lo, d_h1_hi = _bounds_v(d_h1)
        d_h2_lo, d_h2_hi = _bounds_v(d_h2)
        d_ang_lo, d_ang_hi = _bounds_deg(d_ang)
        p_h1_lo, p_h1_hi = _bounds_v(p_h1)
        p_h2_lo, p_h2_hi = _bounds_v(p_h2)
        p_ang_lo, p_ang_hi = _bounds_deg(p_ang)

        # --- Clip using non-switch examples ---
        d_h1_lo, d_h1_hi, d_h1_nb, d_h1_na = _clip_with_negatives(d_h1_lo, d_h1_hi, float(np.mean(d_h1)), ns_h1)
        d_h2_lo, d_h2_hi, d_h2_nb, d_h2_na = _clip_with_negatives(d_h2_lo, d_h2_hi, float(np.mean(d_h2)), ns_h2)
        d_ang_lo, d_ang_hi, d_ang_nb, d_ang_na = _clip_with_negatives(
            d_ang_lo, d_ang_hi, float(np.mean(d_ang)), ns_ang, is_degrees=True
        )

        p_h1_lo, p_h1_hi, p_h1_nb, p_h1_na = _clip_with_negatives(p_h1_lo, p_h1_hi, float(np.mean(p_h1)), ns_h1)
        p_h2_lo, p_h2_hi, p_h2_nb, p_h2_na = _clip_with_negatives(p_h2_lo, p_h2_hi, float(np.mean(p_h2)), ns_h2)
        p_ang_lo, p_ang_hi, p_ang_nb, p_ang_na = _clip_with_negatives(
            p_ang_lo, p_ang_hi, float(np.mean(p_ang)), ns_ang, is_degrees=True
        )

        thresholds = {
            "dorsiflexion": {
                "hall1_lower": d_h1_lo,
                "hall1_upper": d_h1_hi,
                "hall2_lower": d_h2_lo,
                "hall2_upper": d_h2_hi,
                "hall1_dot_upper": _dot_upper(d_h1_dot),
                "angle_lower": d_ang_lo,
                "angle_upper": d_ang_hi,
                "angle_dot_sign": 1 if float(np.mean(d_ang_dot)) >= 0 else -1,
                "n_events": len(dorsi_events),
            },
            "plantarflexion": {
                "hall1_lower": p_h1_lo,
                "hall1_upper": p_h1_hi,
                "hall2_lower": p_h2_lo,
                "hall2_upper": p_h2_hi,
                "angle_lower": p_ang_lo,
                "angle_upper": p_ang_hi,
                "angle_dot_sign": 1 if float(np.mean(p_ang_dot)) >= 0 else -1,
                "n_events": len(plantar_events),
            },
        }

        with open(self.hall_threshold_path, "w") as f:
            json.dump(thresholds, f, indent=2)

        def _clip_note(nb: int, na: int) -> str:
            if nb == 0 and na == 0:
                return ""
            parts = []
            if nb:
                parts.append(f"{nb} clipped below")
            if na:
                parts.append(f"{na} clipped above")
            return f"  ← {', '.join(parts)}"

        d = thresholds["dorsiflexion"]
        p = thresholds["plantarflexion"]
        nn = len(no_switch_events)
        print(f"\nThresholds saved to {self.hall_threshold_path}")
        print(
            f"  (n_std_hall={n_std_hall}  n_std_angle={n_std_angle}"
            f"  neg_exclusion_pct={neg_exclusion_pct}  n_negatives={nn})"
        )
        print(f"  Dorsiflexion  ({d['n_events']} events):")
        print(
            f"    hall1  [{d['hall1_lower']:.4f}, {d['hall1_upper']:.4f}] V"
            f"  (mean={np.mean(d_h1):.4f} sigma={np.std(d_h1):.4f}){_clip_note(d_h1_nb, d_h1_na)}"
        )
        print(
            f"    hall2  [{d['hall2_lower']:.4f}, {d['hall2_upper']:.4f}] V"
            f"  (mean={np.mean(d_h2):.4f} sigma={np.std(d_h2):.4f}){_clip_note(d_h2_nb, d_h2_na)}"
        )
        print(
            f"    angle  [{d['angle_lower']:.2f}, {d['angle_upper']:.2f}]deg"
            f"  (mean={np.mean(d_ang):.2f} sigma={np.std(d_ang):.2f}){_clip_note(d_ang_nb, d_ang_na)}"
        )
        print(f"    d(h1)_upper = {d['hall1_dot_upper']:.4f}")
        print(f"  Plantarflexion ({p['n_events']} events):")
        print(
            f"    hall1  [{p['hall1_lower']:.4f}, {p['hall1_upper']:.4f}] V"
            f"  (mean={np.mean(p_h1):.4f} sigma={np.std(p_h1):.4f}){_clip_note(p_h1_nb, p_h1_na)}"
        )
        print(
            f"    hall2  [{p['hall2_lower']:.4f}, {p['hall2_upper']:.4f}] V"
            f"  (mean={np.mean(p_h2):.4f} sigma={np.std(p_h2):.4f}){_clip_note(p_h2_nb, p_h2_na)}"
        )
        print(
            f"    angle  [{p['angle_lower']:.2f}, {p['angle_upper']:.2f}]deg"
            f"  (mean={np.mean(p_ang):.2f} sigma={np.std(p_ang):.2f}){_clip_note(p_ang_nb, p_ang_na)}"
        )
        LOGGER.info(f"Hall switch thresholds saved to {self.hall_threshold_path}")

        return thresholds

    def load_hall_thresholds(self) -> dict[str, dict[str, Any]]:
        """
        Load hall-effect switch thresholds from file.

        Returns:
            Threshold dict with 'dorsiflexion' and 'plantarflexion' keys.

        Raises:
            FileNotFoundError: If the threshold file does not exist.
        """
        if not self.hall_threshold_path.exists():
            raise FileNotFoundError(
                f"No hall threshold file found at {self.hall_threshold_path}. " "Run calibrate_hall_switches() first."
            )
        with open(self.hall_threshold_path) as f:
            thresholds: dict[str, dict[str, Any]] = json.load(f)
        LOGGER.info(f"Loaded hall switch thresholds from {self.hall_threshold_path}")
        return thresholds


if __name__ == "__main__":
    pass
