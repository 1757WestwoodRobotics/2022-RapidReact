import commands2

import constants

from .drivedistance import DriveDistance

from subsystems.drivesubsystem import DriveSubsystem


class ComplexAuto(commands2.SequentialCommandGroup):
    """
    A complex auto command that drives forward, and then drives backward.
    """

    def __init__(self, drive: DriveSubsystem):
        super().__init__(
            # Drive forward the specified distance
            DriveDistance(
                constants.kAutoDriveDistanceInches, constants.kAutoDriveSpeed, drive
            ),
            # Drive backward the specified distance
            DriveDistance(
                constants.kAutoBackupDistanceInches, -constants.kAutoDriveSpeed, drive
            ),
        )
