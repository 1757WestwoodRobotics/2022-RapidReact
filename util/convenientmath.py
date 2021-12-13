import math
from wpilib.geometry import Rotation2d, Translation2d


def clamp(input: float, minimum: float, maximum: float) -> float:
    return max(min(input, maximum), minimum)


def normalizeRotation(input: Rotation2d) -> Rotation2d:
    """
    Normalize the given rotation to the range [-pi, pi)
    """
    inputAngle = input.radians()
    return Rotation2d(
        inputAngle - 2 * math.pi * math.floor((inputAngle + math.pi) / (2 * math.pi))
    )


def translationFromDistanceAndRotation(
    distance: float, rotation: Rotation2d
) -> Translation2d:
    return Translation2d(distance * rotation.cos(), distance * rotation.sin())


def rotationFromTranslation(translation: Translation2d) -> Rotation2d:
    return Rotation2d(math.atan2(translation.Y(), translation.X()))
