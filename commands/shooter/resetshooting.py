from commands2 import CommandBase
from wpimath.geometry import Rotation2d

from subsystems.shootersubsystem import ShooterSubsystem


class ResetShooting(CommandBase):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.addRequirements([self.shooter])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.shooter.setAsStartingPosition()
        self.shooter.rotateTurret(Rotation2d.fromDegrees(0))

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
