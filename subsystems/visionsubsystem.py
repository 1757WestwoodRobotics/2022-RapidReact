import math
import typing

from commands2 import SubsystemBase
from networktables import NetworkTables
from wpilib import SmartDashboard, Timer
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
import constants
from util import convenientmath


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


class BallTrackingModule:
    def __init__(self) -> None:
        self.targetAngle = None
        self.targetDistance = None
        self.limelightCargoNetworkTable = NetworkTables.getTable(
            constants.kLimelightCargoNetworkTableName
        )

    def getTargetAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetAngle

    def getTargetDistance(self) -> typing.Optional[float]:
        return self.targetDistance

    def update(self) -> None:
        NetworkTables.initialize()

        if self.limelightCargoNetworkTable.getBoolean(
            constants.kLimelightTargetValidKey, False
        ):
            ballAngleYToCamera = Rotation2d.fromDegrees(
                self.limelightCargoNetworkTable.getValue(
                    constants.kLimelightTargetVerticalAngleKey, 0
                )
            )

            distanceToCamera = (
                Rotation2d(
                    constants.kIntakeCameraTiltAngle.radians()
                    + ballAngleYToCamera.radians()
                ).tan()
                * constants.kIntakeCameraHeightInMeters
            )

            if not constants.kIsIntakeCameraCentered:
                ballAngleXToCamera = self.limelightCargoNetworkTable.getValue(
                    constants.kLimelightTargetHorizontalAngleKey, float("inf")
                )
                supplementaryAngleToCamera = Rotation2d.fromDegrees(
                    180 - ballAngleXToCamera
                )

                self.targetDistance = math.sqrt(
                    constants.kIntakeCameraCenterOffsetInMeters**2
                    + distanceToCamera**2
                    - 2
                    * constants.kIntakeCameraCenterOffsetInMeters
                    * distanceToCamera
                    * supplementaryAngleToCamera.cos()
                )

                self.targetAngle = Rotation2d(
                    math.asin(
                        (supplementaryAngleToCamera.sin() * distanceToCamera)
                        / self.targetDistance
                    )
                )
            else:
                self.targetDistance = distanceToCamera
                self.targetAngle = Rotation2d.fromDegrees(
                    self.limelightCargoNetworkTable.getValue(
                        constants.kLimelightTargetHorizontalAngleKey
                    )
                )

        else:
            self.targetAngle = None
            self.targetDistance = None

        if self.targetAngle is not None:
            SmartDashboard.putNumber(
                constants.kBallAngleRelativeToRobotKeys.valueKey,
                convenientmath.normalizeRotation(self.targetAngle).radians(),
            )
            SmartDashboard.putBoolean(
                constants.kBallAngleRelativeToRobotKeys.validKey, True
            )
        else:
            SmartDashboard.putBoolean(
                constants.kBallAngleRelativeToRobotKeys.validKey, False
            )

        if self.targetDistance is not None:
            SmartDashboard.putNumber(
                constants.kBallDistanceRelativeToRobotKeys.valueKey, self.targetDistance
            )
            SmartDashboard.putBoolean(
                constants.kBallDistanceRelativeToRobotKeys.validKey, True
            )
        else:
            SmartDashboard.putBoolean(
                constants.kBallDistanceRelativeToRobotKeys.validKey, False
            )


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

        self.servoController = PIDController(
            constants.kCameraServoPGain,
            constants.kCameraServoIGain,
            constants.kCameraServoDGain,
        )

        NetworkTables.initialize()
        self.limelightNetworkTable = NetworkTables.getTable(
            constants.kLimelightNetworkTableName
        )

    def getTargetAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetAngle

    def getTargetDistance(self) -> typing.Optional[float]:
        return self.targetDistance

    def getTargetFacingAngle(self) -> typing.Optional[Rotation2d]:
        return self.targetFacingAngle

    def update(self) -> None:
        targetValid = self.limelightNetworkTable.getNumber(
            constants.kLimelightTargetValidKey, constants.kLimelightTargetInvalidValue
        )
        if targetValid:
            # real model has limelight rotated 90 degrees
            self.targetAngle = Rotation2d.fromDegrees(
                self.limelightNetworkTable.getNumber(
                    constants.kLimelightTargetVerticalAngleKey, 0.0
                )
            )
            self.targetDistance = (
                constants.kSimDefaultTargetHeight - constants.kLimelightVerticalOffset
            ) / (
                Rotation2d.fromDegrees(
                    self.limelightNetworkTable.getNumber(
                        constants.kLimelightTargetHorizontalAngleKey, 0.0
                    )
                )
                + constants.kLimelightAngleOffset
            ).tan()
        else:
            self.targetDistance = None
            self.targetAngle = None

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

        self.ballTrackingModule = BallTrackingModule()

        self.printTimer = Timer()

    def resetTrackingModule(self):
        self.trackingModule.reset()

    def periodic(self):
        """
        Called periodically by the command framework. Updates the estimate of the target's pose from the tracking data.
        """
        self.trackingModule.update()
        self.ballTrackingModule.update()

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

            limelightPanAngle = (
                SmartDashboard.getNumber(constants.kShootingTurretAngleKey, 0)
                * constants.kRadiansPerDegree
            )

            robotPose = Pose2d(
                robotPoseX, robotPoseY, robotPoseAngle + limelightPanAngle
            )
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

        if self.printTimer.hasPeriodPassed(constants.kPrintPeriod):
            print(
                f"ta: {self.trackingModule.getTargetAngle().degrees():.0f}* td: {self.trackingModule.getTargetDistance():.1f} tfa: {self.trackingModule.getTargetFacingAngle().degrees():.0f}*"
            )
