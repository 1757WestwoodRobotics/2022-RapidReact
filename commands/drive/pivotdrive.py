import typing
from commands2 import CommandBase
from wpilib import Preferences
from subsystems.drivesubsystem import DriveSubsystem


class PivotDrive(
    CommandBase
):  # Pivot drives constantly rotates around a center point. For every vertical "foward" amount maintains a constant rotation
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        swivel: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.swivel = swivel

        self.addRequirements([self.drive])
        self.setName(__class__.__name__)
        Preferences.initFloat("Robot Relative Sensitivity", 0.2)

    def execute(self) -> None:
        curve = 0
        if abs(self.forward()) > 0:
            curve = -self.forward() * self.swivel()

        self.drive.arcadeDriveWithFactors(
            0,
            self.swivel(),
            curve,
            DriveSubsystem.CoordinateMode.RobotRelative,
        )
