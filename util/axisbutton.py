from typing import Callable
from commands2.button import Button


Axis = Callable[[], float]


class AxisButton(Button):
    """a trigger that can be fired by an axis hitting a certain limit"""

    def __init__(self, axis: Axis, threshold: float) -> None:
        self.axis = axis
        self.threshold = threshold
        super().__init__(lambda:  self.axis() > self.threshold)

