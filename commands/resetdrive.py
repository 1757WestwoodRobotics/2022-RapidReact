from commands2 import CommandBase

from subsystems.drivesubsystem import DriveSubsystem


class ResetDrive(CommandBase):
    def __init__(self, drive: DriveSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.drive.resetSwerveModules()

    # pylint: disable=unused-argument,no-self-use
    def end(self, interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
