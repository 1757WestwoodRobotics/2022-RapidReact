from commands2 import CommandBase
from wpimath.geometry import Rotation2d

from subsystems.shootingsubsystem import ShootingSubsystem


class ResetShooting(CommandBase):
    def __init__(self, shooter: ShootingSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.addRequirements([self.shooter])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.shooter.setAsStartingPosition()
        self.shooter.rotateTurret(Rotation2d.fromDegrees(0))

    # pylint: disable-next=no-self-use
    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
