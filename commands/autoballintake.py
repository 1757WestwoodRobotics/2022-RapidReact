from commands2 import CommandBase
from networktables import NetworkTables
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpilib.geometry import Rotation2d
from wpilib import SmartDashboard

import constants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from util import convenientmath


class AutoBallIntake(CommandBase):
    def __init__(self, drive: DriveSubsystem, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.intake = intake
        self.distanceController = ProfiledPIDController(
            constants.kDriveToBallPGain,
            constants.kDriveToBallIGain,
            constants.kDriveToBallDGain,
            TrapezoidProfile.Constraints(
                constants.kMaxForwardLinearVelocity,
                constants.kMaxForwardLinearAcceleration,
            ),
        )
        self.ballAngle = Rotation2d()
        self.distanceController.setTolerance(  # These constants are reusable and likely will not need to change
            constants.kDriveToTargetDistanceTolerance,
            constants.kDriveToTargetLinearVelocityTolerance,
        )
        self.ballDistance = 0
        self.addRequirements([self.drive, self.intake])

        self.angleController = ProfiledPIDControllerRadians(
            constants.kDriveToTargetAnglePGain,
            constants.kDriveToTargetAngleIGain,
            constants.kDriveToTargetAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.angleController.setTolerance(  # bet you can't guess what's copy/pasted
            constants.kDriveToTargetAngleTolerance,
            constants.kDriveToTargetAngularVelocityTolerance,
        )
        self.angleController.enableContinuousInput(
            -1 * constants.kRadiansPerRevolution / 2,
            constants.kRadiansPerRevolution / 2,
        )

    def updateDistanceToGoal(self) -> None:
        if not (
            SmartDashboard.getBoolean(
                constants.kBallAngleRelativeToRobotKeys.validKey, False
            )
            and SmartDashboard.getBoolean(
                constants.kBallDistanceRelativeToRobotKeys.validKey, False
            )
        ):
            self.ballDistance = 0
            self.ballAngle = Rotation2d()
            return

        self.ballDistance = SmartDashboard.getNumber(
            constants.kBallDistanceRelativeToRobotKeys.valueKey, 0
        )

        self.ballAngle = Rotation2d(
            SmartDashboard.getNumber(
                constants.kBallAngleRelativeToRobotKeys.valueKey, 0
            )
        )

    def initialize(self) -> None:
        CommandBase.initialize(self)

        self.updateDistanceToGoal()

        self.distanceController.reset(self.ballDistance)
        self.angleController.reset(self.ballAngle.radians())

    def execute(self) -> None:
        if NetworkTables.getTable(constants.kLimelightCargoNetworkTableName).getBoolean(
            constants.kLimelightTargetValidKey, False
        ):
            self.updateDistanceToGoal()

            distanceControllerOutput = self.distanceController.calculate(
                -1 * self.ballDistance, 0
            )

            angleControllerOutput = self.angleController.calculate(
                -1 * self.ballAngle.radians(), 0
            )

            distanceControllerAxisOutputs = (
                convenientmath.translationFromDistanceAndRotation(
                    distanceControllerOutput, self.ballAngle
                )
            )

            self.drive.arcadeDriveWithFactors(
                distanceControllerAxisOutputs.X(),
                distanceControllerAxisOutputs.Y(),
                angleControllerOutput,
                DriveSubsystem.CoordinateMode.RobotRelative,
            )

            self.intake.deployIntake()
        else:
            self.intake.retractIntake()

    def end(self, _interrupted: bool) -> None:
        self.drive.arcadeDriveWithFactors(
            0, 0, 0, DriveSubsystem.CoordinateMode.RobotRelative
        )
        self.intake.retractIntake()

    def isFinished(self) -> bool:
        return self.distanceController.atGoal() and self.angleController.atGoal()
