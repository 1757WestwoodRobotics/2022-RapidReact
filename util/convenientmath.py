import math
import typing
from wpimath.geometry import Rotation2d, Translation2d, Pose2d

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


def rotateAroundPoint(
    pose: Pose2d, position: Translation2d, rotation: Rotation2d
) -> Pose2d:
    deltaTranslation = pose.translation() - position
    newRotation = rotation + pose.rotation()

    rotatedTranslation = translationFromDistanceAndRotation(
        deltaTranslation.distance(Translation2d()),
        rotationFromTranslation(deltaTranslation) + rotation,
    )

    return Pose2d(rotatedTranslation + position, newRotation)


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


class Interpolator:
    def __init__(self, vals: typing.List[typing.Tuple[number, number]]) -> None:
        """uses a list of tuples of x/y coordinates, to interpolate linearly between them"""
        self.vals = vals

    def interpolate(self, val: number):
        """samples at an x position the interpolated linear value between the two closest defined points"""
        smallest = (0, 0)
        greatest = (math.inf, math.inf)
        for key in self.vals:
            if smallest[0] < key[0] and key[0] <= val:
                smallest = key

            if val <= key[0] and key[0] < greatest[0]:
                greatest = key

        return map_range(val, smallest[0], greatest[0], smallest[1], greatest[1])
