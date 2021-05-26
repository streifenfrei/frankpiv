import os

import frankpiv.frankpyv

script_directory = os.path.dirname(os.path.abspath(__file__))


class Controller:
    def __init__(self, config_file=None):
        if config_file is None:
            config_file = os.path.join(script_directory, "default_config.yml")
        self.controller = frankpiv.frankpyv.Controller(config_file)

    def __enter__(self):
        self.controller.start()
        return self.controller

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.controller.stop()

