from os import path
from wpimath.trajectory import Trajectory
from pathplannerlib import PathPlanner

import constants


def trajectoryFromFile(name: str) -> Trajectory:
    return PathPlanner.loadPath(
        path.join(
            path.dirname(path.realpath(__file__)),
            "..",
            "..",
            "deploy",
            "pathplanner",
            name
        ),
        constants.kMaxForwardLinearVelocity,
        constants.kMaxForwardLinearAcceleration,
    ).asWPILibTrajectory()
