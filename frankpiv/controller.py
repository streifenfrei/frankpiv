import importlib
import os

import yaml

from frankpiv.backend.general import GeneralBackend

script_directory = os.path.dirname(os.path.abspath(__file__))


class Controller:

    def __init__(self, config: dict):
        assert "backend" in config
        assert "eef_ppoint_distance" in config
        assert "tool_length" in config
        assert config["eef_ppoint_distance"] < config["tool_length"]
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
    def from_file(cls, config_file=None):
        if config_file is None:
            config_file = os.path.join(script_directory, "default_config.yml")
        with open(config_file, "r") as config_file:
            config = yaml.load(config_file.read(), Loader=yaml.FullLoader)
        return cls(config)
