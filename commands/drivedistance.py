import commands2

from subsystems.drivesubsystem import DriveSubsystem


class DriveDistance(commands2.CommandBase):
    def __init__(self, distance: float, speedFactor: float, drive: DriveSubsystem) -> None:
        super().__init__()
        self.distance = distance
        self.speedFactor = speedFactor
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self) -> None:
        self.drive.resetEncoders()
        self.drive.arcadeDrive(self.speedFactor, 0)

    def execute(self) -> None:
        self.drive.arcadeDrive(self.speedFactor, 0)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        return abs(self.drive.getAverageEncoderDistance()) >= self.distance
