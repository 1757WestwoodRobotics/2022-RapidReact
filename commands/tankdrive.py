import typing
from commands2 import CommandBase
from subsystems.drivesubsystem import DriveSubsystem
import constants


class TankDrive(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        left: typing.Callable[[], float],
        right: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.left = left
        self.right = right

        self.addRequirements([self.drive])

    def execute(self) -> None:
        l = self.left()
        r = self.right()
        self.drive.arcadeDriveWithFactors(
            l + r,
            0,
            (r - l) * constants.kRobotWidth,
            DriveSubsystem.CoordinateMode.RobotRelative,
        )
