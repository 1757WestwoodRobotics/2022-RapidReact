import math
import typing
from wpilib.geometry import Rotation2d, Translation2d

number = typing.Union[float, int]


def clamp(inputValue: float, minimum: float, maximum: float) -> float:
    return max(min(inputValue, maximum), minimum)


def normalizeRotation(inputRotation: Rotation2d) -> Rotation2d:
    """
    Normalize the given rotation to the range [-pi, pi)
    """
    inputAngle = inputRotation.radians()
    return Rotation2d(
        inputAngle - 2 * math.pi * math.floor((inputAngle + math.pi) / (2 * math.pi))
    )


def translationFromDistanceAndRotation(
    distance: float, rotation: Rotation2d
) -> Translation2d:
    return Translation2d(distance * rotation.cos(), distance * rotation.sin())


def rotationFromTranslation(translation: Translation2d) -> Rotation2d:
    return Rotation2d(math.atan2(translation.Y(), translation.X()))


def map_range(
    value: number,
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number,
):
    return (value - inputMin) * (outputMax - outputMin) / (
        inputMax - inputMin
    ) + outputMin
