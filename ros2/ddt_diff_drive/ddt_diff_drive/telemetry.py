"""Latest values reported by the board. Outlives a reconnect: LoggingBridge is recreated, this is not."""

from ddt_diff_drive.ddt_bridge import Feedback


class BridgeTelemetry:
    def __init__(self):
        # motor id -> (latest Feedback, time.monotonic() when it was received)
        self.feedbacks: dict[int, tuple[Feedback, float]] = {}

    def record(self, feedback: Feedback, received_at: float) -> None:
        self.feedbacks[feedback.motor_id] = (feedback, received_at)
