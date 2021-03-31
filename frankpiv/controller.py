import importlib
import os

import yaml

from frankpiv.backend.general import GeneralBackend

script_directory = os.path.dirname(os.path.abspath(__file__))


class Controller:

    def __init__(self, config: dict):
        assert config["eef_ppoint_distance"] < config["tool_length"]
        config["max_angle"] = min([config["max_angle"], 90])
        config["roll_boundaries"][0] = max([config["roll_boundaries"][0], -360])
        config["roll_boundaries"][1] = min([config["roll_boundaries"][1], 360])
        config["z_translation_boundaries"][0] = max([config["z_translation_boundaries"][0],
                                                     config["eef_ppoint_distance"] - config["tool_length"]])
        config["z_translation_boundaries"][1] = min([config["z_translation_boundaries"][1],
                                                     config["eef_ppoint_distance"]])
        try:
            backend_module = importlib.import_module(f"frankpiv.backend.{config['backend']}")
            self.backend = backend_module.Backend(config)
        except (ImportError, AttributeError):
            raise ValueError(f"No such backend: {config['backend']}")

    def __enter__(self) -> GeneralBackend:
        return self.backend.__enter__()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.backend.__exit__(exc_type, exc_val, exc_tb)

    @classmethod
    def from_file(cls, config_file: str = None):
        if config_file is None:
            config_file = os.path.join(script_directory, "default_config.yml")
        with open(config_file, "r") as config_file:
            config = yaml.load(config_file.read(), Loader=yaml.FullLoader)
        return cls(config)
