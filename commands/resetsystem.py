from commands2 import ParallelCommandGroup
from commands.resetdrive import ResetDrive
from commands.shooter.resetshooting import ResetShooting
from subsystems.drivesubsystem import DriveSubsystem

from subsystems.shootingsubsystem import ShootingSubsystem


class ResetSystem(ParallelCommandGroup):
    def __init__(self, shooting: ShootingSubsystem, drive: DriveSubsystem):
        super().__init__(ResetDrive(drive), ResetShooting(shooting))
        self.setName(__class__.__name__)
