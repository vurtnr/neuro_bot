TERMINAL_STAGES = {"success", "failed"}
ACTIVE_STAGES = {
    "waiting_for_qr",
    "qr_detected",
    "ble_connecting",
    "querying_device",
}


class InspectionGate:
    def __init__(self, require_active_session: bool):
        self.require_active_session = require_active_session
        self.active_request_id: str | None = None

    def should_publish(self) -> bool:
        if not self.require_active_session:
            return True
        return self.active_request_id is not None

    def update(self, request_id: str, stage: str) -> None:
        if not self.require_active_session:
            return

        if stage in ACTIVE_STAGES:
            self.active_request_id = request_id or self.active_request_id
        elif stage in TERMINAL_STAGES and request_id == self.active_request_id:
            self.active_request_id = None
