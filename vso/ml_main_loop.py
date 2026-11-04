import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

from opensourceleg.robots.vso import VSO
from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging import LOGGER
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.base import SensorBase
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop




FREQUENCY = 200
DT = 1 / FREQUENCY
OFFLINE = True
PREDICT_EVERY = 50      # loops between predictions; samples are still collected every loop
CALIBRATION_CYCLES = 1000  # 5 s at 200 Hz
ML_TESTING = True  # If True, ML model is loaded and run on the Pi; if False, ML model is not called.


# All three models share the same ONNX signature: window (batch, 5, 1200) -> logits (batch, 5)
MODEL = "conv2d"          # "tcn" | "linear" | "conv2d"
MODEL_DIR = Path(__file__).resolve().parent / "trained_Models"
MODEL_PATHS = {
    "tcn": MODEL_DIR / "VSO_tcn.onnx",
    "linear": MODEL_DIR / "VSO_linear.onnx",
    "conv2d": MODEL_DIR / "VSO_conv2d.onnx",
}

WINDOW_SIZE = 1200  # 6 s at 200 Hz
STIFFNESS_NM_PER_RAD = 0.0  # TODO: set to the orthosis's real stiffness. 0 is outside the trained range.
ACTIVITY_LIST = ["FLAT", "INCLINE", "DECLINE", "STAIR ASCENT", "STAIR DESCENT"]

# Training feature order: ankle (deg), foot sagittal (deg), shank sagittal (deg), stiffness (Nm/rad), foot accel Y.
FEATURE_MIN = np.array([-30.0, -45.0, -60.0, 44.839, -4.0])
FEATURE_MAX = np.array([30.0, 45.0, 60.0, 246.531, 4.0])


def calibrate_imu(vso: VSO) -> tuple[float, float, float, float]:
    """Calibrate the IMUs by averaging pitch and roll over a number of cycles."""
    foot_imu = vso.sensors["foot_imu"]
    shank_imu = vso.sensors["shank_imu"]

    foot_pitch_offset_acc = 0.0
    shank_pitch_offset_acc = 0.0
    foot_roll_offset_acc = 0.0
    shank_roll_offset_acc = 0.0

    LOGGER.info(f"Calibrating IMUs over {CALIBRATION_CYCLES} cycles. Hold still...")
    for _ in range(CALIBRATION_CYCLES):
        vso.update()
        foot_pitch_offset_acc += foot_imu.pitch
        shank_pitch_offset_acc += shank_imu.pitch
        foot_roll_offset_acc += foot_imu.roll
        shank_roll_offset_acc += shank_imu.roll

    foot_pitch_offset = foot_pitch_offset_acc / CALIBRATION_CYCLES
    shank_pitch_offset = shank_pitch_offset_acc / CALIBRATION_CYCLES
    foot_roll_offset = foot_roll_offset_acc / CALIBRATION_CYCLES
    shank_roll_offset = shank_roll_offset_acc / CALIBRATION_CYCLES

    LOGGER.info(
        f"Calibration done. foot_pitch={foot_pitch_offset:.4f} shank_pitch={shank_pitch_offset:.4f} "
        f"foot_roll={foot_roll_offset:.4f} shank_roll={shank_roll_offset:.4f} rad"
    )
    return foot_pitch_offset, shank_pitch_offset, foot_roll_offset, shank_roll_offset


def read_features(
    vso: VSO,
    foot_pitch_offset: float,
    shank_pitch_offset: float,
    foot_roll_offset: float,
    shank_roll_offset: float,
) -> np.ndarray:
    """Return the current sample in training feature order."""
    foot_imu = vso.sensors["foot_imu"]
    shank_imu = vso.sensors["shank_imu"]

    return np.array([
        np.rad2deg((shank_imu.roll - shank_roll_offset) - (foot_imu.roll - foot_roll_offset)),
        np.rad2deg(foot_imu.pitch - foot_pitch_offset),
        np.rad2deg(shank_imu.pitch - shank_pitch_offset),
        STIFFNESS_NM_PER_RAD,
        foot_imu.acc_y,  # Not divided by 9.81: the training data's "g" column holds the IMU's raw value.
    ])


def normalize_window(window: np.ndarray) -> np.ndarray:
    """Crop a (5, WINDOW_SIZE) window to the feature ranges, then scale each feature to [-1, 1]."""
    feature_min = FEATURE_MIN[:, np.newaxis]
    feature_max = FEATURE_MAX[:, np.newaxis]
    cropped = np.clip(window, feature_min, feature_max)
    return (2 * (cropped - feature_min) / (feature_max - feature_min) - 1).astype(np.float32)


def load_model(model_path: Path) -> ort.InferenceSession:
    """Load the ONNX model once, before the control loop starts."""
    if not model_path.exists():
        raise FileNotFoundError(f"ONNX model not found: {model_path}")

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1  # Leave the other CPU cores free for the 200 Hz loop.
    session = ort.InferenceSession(str(model_path), options, providers=["CPUExecutionProvider"])
    LOGGER.info(f"Loaded {MODEL} model from {model_path}")
    return session


def predict_activity(session: ort.InferenceSession, window: np.ndarray, prediction: dict) -> None:
    """Run one prediction and store it in `prediction`. Prints on a change of activity, and at least once a second."""
    model_input = normalize_window(window)[np.newaxis]  # (1, 5, WINDOW_SIZE)
    logits = session.run(None, {session.get_inputs()[0].name: model_input})[0][0]
    probabilities = np.exp(logits - logits.max())
    probabilities /= probabilities.sum()

    best = int(np.argmax(probabilities))
    activity = best + 1  # 1-5, because 0 means no prediction yet
    changed = activity != prediction["activity"]
    prediction["activity"] = activity
    prediction["confidence"] = float(probabilities[best])

    now = time.perf_counter()
    if changed or now - prediction["last_print"] >= 1.0:
        prediction["last_print"] = now
        others = ", ".join(
            f"{ACTIVITY_LIST[i]}:{probabilities[i]:.3f}" for i in range(len(ACTIVITY_LIST)) if i != best
        )
        LOGGER.info(f"Predicted:{ACTIVITY_LIST[best]}, probability:{probabilities[best]:.3f}, others: {others}")


def setup_logger(datalog: Logger, features: np.ndarray, prediction: dict) -> None:
    """Register all CSV columns with the data logger. Call once, before the first datalog.update()."""
    start_time = time.time()

    axes = [
        (lambda: time.time() - start_time, "Time (s)"),
        (lambda: features[3], "Stiffness (Nm/rad)"),
        (lambda: features[0], "Ankle Angle IMU (deg)"),
        (lambda: features[1], "Foot Sagittal Angle (deg)"),
        (lambda: features[2], "Shank Sagittal Angle (deg)"),
        (lambda: features[4], "Accel Y Foot (m/s^2)"),
        (lambda: prediction["activity"], "Activity Label"),
    ]

    funcs, labels = zip(*axes)
    datalog.track_function(list(funcs), list(labels))


if __name__ == '__main__':
    datalog = Logger(
        log_path="./logs",
        file_name="ML_logger.log",
    )

    LOGGER.info("Finished initializing data logger...")

    vso = VSO[MaxonActuator, SensorBase](
        tag="variableStiffessOrthosis",
        actuators={
        },
        sensors={
            "foot_imu": LordMicrostrainIMU(
                tag="foot_imu",
                port="/dev/ttyAMA2",#uart2
                baud_rate=921600, 
                frequency=FREQUENCY,
                update_timeout=500,
                max_packets=1,
                return_packets=False,
                offline=OFFLINE,
            ),
            "shank_imu": LordMicrostrainIMU(
                tag="shank_imu",
                port="/dev/ttyAMA0",#uart0
                baud_rate=921600,  
                frequency=FREQUENCY,
                update_timeout=500,
                max_packets=1,
                return_packets=False,
                offline=OFFLINE,
            ),
        },
    )

 
    LOGGER.info("Finished setting up VSO...")

    
    session = load_model(MODEL_PATHS[MODEL]) if ML_TESTING else None
    if not FEATURE_MIN[3] <= STIFFNESS_NM_PER_RAD <= FEATURE_MAX[3]:
        LOGGER.warning(f"STIFFNESS_NM_PER_RAD={STIFFNESS_NM_PER_RAD} is outside the trained range; predictions will be unreliable.")
    datalog.info("Starting VSO initialization sequence...")

    features = np.zeros(len(FEATURE_MIN))
    window = np.zeros((len(FEATURE_MIN), WINDOW_SIZE))
    prediction = {"activity": 0, "confidence": 0.0, "last_print": 0.0}
    samples_collected = 0

    with vso, datalog:
        LOGGER.info("calibrating imu offset, press enter to continue...")
        input()
        foot_pitch_offset, shank_pitch_offset, foot_roll_offset, shank_roll_offset = calibrate_imu(vso)
        setup_logger(datalog, features, prediction)
        LOGGER.info(f"Collecting {WINDOW_SIZE} samples before the first prediction...")

        for _t in SoftRealtimeLoop(dt=DT):
            vso.update()
            features[:] = read_features(vso, foot_pitch_offset, shank_pitch_offset, foot_roll_offset, shank_roll_offset)
            datalog.update()

            # Oldest sample is dropped, newest goes in the last column.
            window[:, :-1] = window[:, 1:]
            window[:, -1] = features
            samples_collected += 1

            if ML_TESTING and samples_collected >= WINDOW_SIZE and samples_collected % PREDICT_EVERY == 0:
                predict_activity(session, window, prediction)
