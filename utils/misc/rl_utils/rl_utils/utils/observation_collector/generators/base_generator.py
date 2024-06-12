from ..collectors import BaseUnit
from abc import ABC, abstractmethod

from typing import List, Type, TypeVar

D = TypeVar("D")


class ObservationGeneratorUnit(BaseUnit, ABC):
    """
    Base class for observation generator units.

    Attributes:
        name (str): The name of the observation generator unit.
        requires (List[BaseUnit]): A list of required base units.
        data_class (Type[D]): The type of data generated by the observation generator unit.
    """

    name: str = ""
    requires: List[BaseUnit] = []
    data_class: Type[D] = D

    @abstractmethod
    def generate(self, obs_dict: dict) -> D:
        """
        Generates the observation data based on the given observation dictionary.

        Args:
            obs_dict (dict): The observation dictionary.

        Returns:
            D: The generated observation data.
        """
        raise NotImplementedError
