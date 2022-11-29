from os import path
from pathplannerlib import PathPlanner, PathPlannerTrajectory

import constants


def trajectoryFromFile(name: str) -> PathPlannerTrajectory:
    return PathPlanner.loadPath(
        path.join(
            path.dirname(path.realpath(__file__)),
            "..",
            "..",
            "deploy",
            "pathplanner",
            name,
        ),
        constants.kMaxForwardLinearVelocity,
        constants.kMaxForwardLinearAcceleration,
    )
