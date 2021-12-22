import typing

from commands2 import SubsystemBase
from networktables import NetworkTables
from wpilib import SmartDashboard, Timer, RobotBase, PWM
from wpilib.simulation import PWMSim
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
import constants
import util.convenientmath as convenientmath


class TrackingModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def getTargetAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def getTargetDistance(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def getTargetFacingAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def update(self) -> None:
        targetAngle = self.getTargetAngle()
        if targetAngle is not None:
            SmartDashboard.putNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey,
                convenientmath.normalizeRotation(targetAngle).radians(),
            )
            SmartDashboard.putBoolean(
                constants.kTargetAngleRelativeToRobotKeys.validKey, True
            )
        else:
            SmartDashboard.putBoolean(
                constants.kTargetAngleRelativeToRobotKeys.validKey, False
            )

        targetDistance = self.getTargetDistance()
        if targetDistance is not None:
            SmartDashboard.putNumber(
                constants.kTargetDistanceRelativeToRobotKeys.valueKey, targetDistance
            )
            SmartDashboard.putBoolean(
                constants.kTargetDistanceRelativeToRobotKeys.validKey, True
            )
        else:
            SmartDashboard.putBoolean(
                constants.kTargetDistanceRelativeToRobotKeys.validKey, False
            )

        targetFacingAngle = self.getTargetFacingAngle()
        if targetFacingAngle is not None:
            SmartDashboard.putNumber(
                constants.kTargetFacingAngleRelativeToRobotKeys.valueKey,
                convenientmath.normalizeRotation(targetFacingAngle).radians(),
            )
            SmartDashboard.putBoolean(
                constants.kTargetFacingAngleRelativeToRobotKeys.validKey, True
            )
        else:
            SmartDashboard.putBoolean(
                constants.kTargetFacingAngleRelativeToRobotKeys.validKey, False
            )

    def reset(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")


class SimTrackingModule(TrackingModule):
    """
    Implementation of TrackingModule designed for ease of simulation:
        Uses externally provided field-relative target pose to calculate simulated tracking data
    """

    def __init__(
        self,
        name: str,
        targetPoseArrayKey: str,
    ) -> None:
        TrackingModule.__init__(self, name)
        self.targetPoseArrayKey = targetPoseArrayKey
        self.targetAngle = Rotation2d()
        self.targetDistance = 0
        self.targetFacingAngle = Rotation2d()

    def getTargetAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetAngle

    def getTargetDistance(self) -> typing.Optional[float]:
        return self.targetDistance

    def getTargetFacingAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetFacingAngle

    def update(self) -> None:
        targetPoseX, targetPoseY, targetAngle = SmartDashboard.getNumberArray(
            constants.kSimTargetPoseArrayKey, [0, 0, 0]
        )
        targetPose = Pose2d(targetPoseX, targetPoseY, targetAngle)

        robotPoseX, robotPoseY, robotPoseAngle = SmartDashboard.getNumberArray(
            constants.kSimRobotPoseArrayKey, [0, 0, 0]
        )
        robotPose = Pose2d(robotPoseX, robotPoseY, robotPoseAngle)

        robotToTarget = Transform2d(robotPose, targetPose)
        self.targetAngle = Rotation2d(robotToTarget.X(), robotToTarget.Y())
        self.targetDistance = robotToTarget.translation().norm()
        self.targetFacingAngle = robotToTarget.rotation()

        TrackingModule.update(self)

    def reset(self) -> None:
        pass


class LimelightTrackingModule(TrackingModule):
    """
    Implementation of TrackingModule designed for use with the Limelight smart-camera
    """

    def __init__(
        self,
        name: str,
    ) -> None:
        TrackingModule.__init__(self, name)
        self.targetAngle = None
        self.targetDistance = None
        self.targetFacingAngle = None

        NetworkTables.initialize()
        self.limelightNetworkTable = NetworkTables.getTable(
            constants.kLimelightNetworkTableName
        )

        self.rotationServo = (
            PWM(constants.kCameraPanServoPWMChannel)
            if RobotBase.isReal()
            else PWMSim(constants.kCameraSimPanServoPWMChannel)
        )

        self.lastReadAngle = None

    def getTargetAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetAngle

    def getTargetDistance(self) -> typing.Optional[float]:
        return self.targetDistance

    def getTargetFacingAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetFacingAngle

    def getServoAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(
            self.rotationServo.getSpeed() * constants.kCameraServoMaxAngle
        )

    def setServoAngle(self, angle: Rotation2d) -> None:
        self.rotationServo.setSpeed(angle.degrees() / constants.kCameraServoMaxAngle)

    def update(self) -> None:
        targetValid = self.limelightNetworkTable.getNumber(
            constants.kLimelightTargetValidKey, constants.kLimelightTargetInvalidValue
        )
        if targetValid:
            self.limelightAngle = Rotation2d.fromDegrees(
                -1
                * self.limelightNetworkTable.getNumber(
                    constants.kLimelightTargetHorizontalAngleKey, 0.0
                )
            )

            self.targetAngle = self.limelightAngle + self.getServoAngle()

            if not self.limelightAngle == self.lastReadAngle:
                self.setServoAngle(self.targetAngle)

                self.lastReadAngle = self.limelightAngle
        else:
            self.targetAngle = None
            # TODO: sweeping motion of the camera, "search for target"

        TrackingModule.update(self)

    def reset(self) -> None:
        pass


class VisionSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.trackingModule = LimelightTrackingModule(
            constants.kLimelightTrackerModuleName
        )

        self.printTimer = Timer()

    def resetTrackingModule(self):
        self.trackingModule.reset()

    def periodic(self):
        """
        Called periodically by the command framework. Updates the estimate of the target's pose from the tracking data.
        """
        self.trackingModule.update()

        targetAngle = self.trackingModule.getTargetAngle()
        if targetAngle is None:
            SmartDashboard.putBoolean(constants.kTargetPoseArrayKeys.validKey, False)
        else:
            targetDistance = self.trackingModule.getTargetDistance()
            if targetDistance is None:
                targetDistance = float("inf")

            targetFacingAngle = self.trackingModule.getTargetFacingAngle()
            if targetFacingAngle is None:
                targetFacingAngle = Rotation2d()

            robotPoseX, robotPoseY, robotPoseAngle = SmartDashboard.getNumberArray(
                constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
            )
            robotPose = Pose2d(robotPoseX, robotPoseY, robotPoseAngle)
            robotToTarget = Transform2d(
                convenientmath.translationFromDistanceAndRotation(
                    targetDistance, targetAngle
                ),
                targetFacingAngle,
            )
            targetPose = robotPose + robotToTarget
            SmartDashboard.putNumberArray(
                constants.kTargetPoseArrayKeys.valueKey,
                [targetPose.X(), targetPose.Y(), targetPose.rotation().radians()],
            )
            SmartDashboard.putBoolean(constants.kTargetPoseArrayKeys.validKey, True)

            SmartDashboard.putNumber(
                constants.kCameraServoRotationNumberKey,
                self.trackingModule.getServoAngle().degrees(),
            )

        if self.printTimer.hasPeriodPassed(constants.kPrintPeriod):
            print(
                "ta: {:.0f}* td: {:.1f} tfa: {:.0f}*".format(
                    self.trackingModule.getTargetAngle().degrees(),
                    self.trackingModule.getTargetDistance(),
                    self.trackingModule.getTargetFacingAngle().degrees(),
                )
            )
