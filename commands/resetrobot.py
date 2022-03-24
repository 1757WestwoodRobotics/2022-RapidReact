from commands2 import ParallelCommandGroup
from commands.resetdrive import ResetDrive
from commands.shooter.resetshooting import ResetShooting
from subsystems.drivesubsystem import DriveSubsystem

from subsystems.shootersubsystem import ShooterSubsystem


class ResetRobot(ParallelCommandGroup):
    def __init__(self, shooting: ShooterSubsystem, drive: DriveSubsystem):
        super().__init__(ResetDrive(drive), ResetShooting(shooting))
        self.setName(__class__.__name__)
