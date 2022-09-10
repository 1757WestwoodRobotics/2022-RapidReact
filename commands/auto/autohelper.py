from os import path
from wpimath.trajectory import Trajectory, TrajectoryUtil


def trajectoryFromFile(name: str) -> Trajectory:
    return TrajectoryUtil.fromPathweaverJson(
        path.join(
            path.dirname(path.realpath(__file__)),
            "..",
            "..",
            "deploy",
            "pathplanner",
            "generatedJSON",
            name,
        )
    )
