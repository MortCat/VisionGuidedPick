"""Explicit test decision adapter. Never invent boxes, masks or model results."""
class AssumedNgAdapter:
    def __init__(self, config):
        if config.get("operation_mode") != "assumed_ng_demo" or config.get("backend") != "assumed_ng":
            raise ValueError("Assumed NG requires explicit matching demo settings")

    def predict(self, frame):
        # Acknowledge a real validated evidence frame. The coordinator only
        # assumes NG after all five views and report save acknowledgement.
        return []
