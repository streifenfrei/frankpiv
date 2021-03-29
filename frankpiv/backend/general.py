from abc import ABC, abstractmethod
from typing import Union

import numpy as np


class GeneralBackend(ABC):

    @abstractmethod
    def __init__(self, config: dict):
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    @abstractmethod
    def move_to_point(self, frame: Union[list, tuple, np.array], point: Union[list, tuple, np.array]):
        pass

    @abstractmethod
    def move_pyrz(self, pjrz: Union[list, tuple, np.array], degrees: bool):
        pass
