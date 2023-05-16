from commands2 import CommandBase
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d

from subsystems.visionsubsystem import VisionSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

import constants


class CubeAimShooterToTarget(CommandBase):
    def __init__(self, vision: VisionSubsystem, shooter: ShooterSubsystem):
        CommandBase.__init__(self)
        self.vision = vision
        self.shooter = shooter
        self.setName(__class__.__name__)
        self.addRequirements([self.vision, self.shooter])

    def execute(self):
        target = self.vision.getNearestTarget()

        cameraToTag = target[1].translation().toTranslation2d()
        robotAngle = Rotation2d(
            SmartDashboard.getNumberArray(constants.kRobotPoseArrayKeys.valueKey, 0)[2]
        )
        turretAngle = self.shooter.getTurretRotation()
        robotToCamera = (
            constants.kLimelightRelativeToRobotTransform.translation()
            .toTranslation2d()
            .rotateBy(robotAngle + turretAngle)
        )

        robotToNode = robotToCamera + cameraToTag + constants.kApriltagToNodeOffset
        distance = robotToNode.distance()

        if SmartDashboard.getBoolean(constants.kAutoWheelSpeedKey, False):
            match SmartDashboard.getString(constants.kNodeLevelKeys.valueKey, "HIGH"):
                case "High":
                    wheelSpeed = constants.kHighCubeMappingFunction(distance)
                case "MID":
                    wheelSpeed = constants.kMidCubeMappingFunction(distance)
                case "HYBRID":
                    wheelSpeed = constants.kHybridCubeMappingFunction(distance)
        else:
            wheelSpeed = constants.kStaticWheelSpeed

        self.shooter.setWheelSpeed(wheelSpeed)

        newTurretAngle = Rotation2d(-robotAngle.radians() + robotToNode.angle().radians())
        self.shooter.rotateTurret(newTurretAngle)
