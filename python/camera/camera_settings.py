from pydantic import BaseModel, Field

class CameraSettings(BaseModel):
    brightness: int = Field(0, ge=-100, le=100, description="Brightness level")
    contrast: float = Field(1.0, ge=0.0, le=3.0, description="Contrast multiplier")
    gamma: float = Field(1.0, ge=0.1, le=5.0, description="Gamma correction factor")
    saturation: float = Field(1.0, ge=0.0, le=3.0, description="Saturation multiplier")


class CameraSettingsManager:
    def __init__(self):
        self.settings = CameraSettings()

    def save_as_json(self, path="camera_settings.json"):
        with open(path, "w") as f:
            f.write(self.settings.model_dump_json(indent=4))

    def load_from_json(self, path="camera_settings.json"):
        with open(path, "r") as f:
            self.settings = CameraSettings.model_validate_json(f.read())

    def get(self):
        return self.settings.model_dump()

    def put(self, data):
        keys = data.keys()
        model = self.settings.model_dump()
        for k in keys:
            if k in model:
                model[k] = data[k]
        self.settings = CameraSettings(**model)
        self.save_as_json()
        return self.settings.model_dump()
