from commands2 import CommandBase
from wpilib import SmartDashboard

from subsystems.shootingsubsystem import ShootingSubsystem
import constants


class TrackTurret(CommandBase):
    def __init__(self, shoot: ShootingSubsystem):
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shoot = shoot
        self.addRequirements(shoot)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        if SmartDashboard.getBoolean(
            constants.kTargetAngleRelativeToRobotKeys.validKey, False
        ):
            angle = SmartDashboard.getNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, 0.0
            )
            print(f"angle from limelight {angle}")
            self.shoot.trackTurret(angle)

        SmartDashboard.putNumber(
            constants.kCameraServoRotationNumberKey,
            self.shoot.getTurretRotation().degrees(),
        )
        print(self.shoot.getTurretRotation())

    def end(self, _interrupted: bool) -> None:
        print(f"... DONE {self.getName()}")
