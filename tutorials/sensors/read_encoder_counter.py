import time

from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.encoderCounter import LS7366R
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop

FREQUENCY = 200  # control/read loop rate (Hz)
READ_FREQUENCY = 10  # console readout rate (Hz)
DT = 1 / FREQUENCY


def read_encoderCounter(data_logger: Logger) -> None:
    encodercounter = LS7366R()
    encodercounter.clear_counter()

    data_logger.track_function(lambda: encodercounter.read_counter(), name="encodercounter")

    readout_interval = FREQUENCY // READ_FREQUENCY

    with encodercounter:
        loop = SoftRealtimeLoop(dt=DT)
        loop_count = 0
        t_start = time.monotonic()

        for loop_count, _ in enumerate(loop):
            encodercounter.update()

            data_logger.update()
            data_logger.flush_buffer()

            if loop_count % readout_interval == 0:
                elapsed = time.monotonic() - t_start
                print(
                    f"\r  t={elapsed:6.1f}s" f"  encodercount={encodercounter.read_counter():+7d} counts",
                    end="",
                    flush=True,
                )


if __name__ == "__main__":
    data_logger = Logger(
        log_path="./logs",
        file_name="read_encodercounter",
    )
    read_encoderCounter(data_logger)
