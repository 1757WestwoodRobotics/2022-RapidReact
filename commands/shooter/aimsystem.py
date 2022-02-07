from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d

from subsystems.shootingsubsystem import ShootingSubsystem
import constants


class AimSystem(CommandBase):
    def __init__(self, shoot: ShootingSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shoot = shoot
        self.addRequirements([self.shoot])

    def execute(self) -> None:
        if SmartDashboard.getBoolean(
            constants.kTargetDistanceRelativeToRobotKeys.validKey, False
        ):
            distance = SmartDashboard.getNumber(
                constants.kTargetDistanceRelativeToRobotKeys.valueKey, 0.0
            )

            targetSpeed = constants.kShootingMappingFunction(distance)
            self.shoot.setWheelSpeed(targetSpeed)

            targetAngle = Rotation2d(constants.kHoodMappingFunction(distance))
            self.shoot.setHoodAngle(targetAngle)

        if SmartDashboard.getBoolean(
            constants.kTargetAngleRelativeToRobotKeys.validKey, False
        ):
            angle = SmartDashboard.getNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, 0.0
            )
            self.shoot.trackTurret(angle)
