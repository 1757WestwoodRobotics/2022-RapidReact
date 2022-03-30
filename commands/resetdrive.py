from commands2 import CommandBase
from wpimath.geometry import Pose2d

from subsystems.drivesubsystem import DriveSubsystem


class ResetDrive(CommandBase):
    def __init__(self, drive: DriveSubsystem, position: Pose2d = Pose2d()) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.position = position
        self.addRequirements(drive)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.drive.resetSwerveModules()
        self.drive.setOdometryPosition(self.position)

    # pylint: disable-next=no-self-use
    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
