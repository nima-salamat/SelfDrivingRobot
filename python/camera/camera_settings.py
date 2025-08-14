from pydantic import BaseModel, Field
import os

class CameraSettings(BaseModel):
    brightness: int = Field(0, ge=-100, le=100, description="Brightness level")
    contrast: float = Field(1.0, ge=0.0, le=3.0, description="Contrast multiplier")
    gamma: float = Field(1.0, ge=0.1, le=5.0, description="Gamma correction factor")
    saturation: float = Field(1.0, ge=0.0, le=3.0, description="Saturation multiplier")


class CameraSettingsManager:
    def __init__(self, path: str = "camera_settings.json"):
        self.path = path
        self.settings = self.load()
        self.save()

    def load(self):
        if os.path.exists(self.path):
            with open(self.path, "r") as f:
                return CameraSettings.model_validate_json(f.read())
        else:
            return CameraSettings()

    def save(self):
        with open(self.path, "w") as f:
            f.write(self.settings.model_dump_json(indent=4))

    def get(self):
        return self.settings.model_dump()

    def put(self, data):
        # model_data = self.settings.model_dump()
        # for k in data:
        #     if k in model_data:
        #         model_data[k] = data[k]
        # self.settings = CameraSettings(**model_data)
        # self.save()
        # return self.settings.model_dump()

        # pydantic automatic (it has validator by itself)
        updated = self.settings.model_copy(update=data)  # validates automatically
        self.settings = updated
        self.save()
        return self.settings.model_dump()
