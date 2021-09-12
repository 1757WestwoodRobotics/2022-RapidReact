from commands2 import SubsystemBase
from wpilib import Encoder, PWMVictorSPX, RobotBase
from ctre import CANCoder, ControlMode, ErrorCode, WPI_TalonFX
from navx import AHRS
from wpimath.geometry import Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

import constants


class SwerveModule:
    def getSwerveAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelLinearVelocity(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.getWheelLinearVelocity(),
            self.getSwerveAngle(),
        )

    def applyState(self, state: SwerveModuleState) -> None:
        optimizedState = SwerveModuleState.optimize(state, self.getSwerveAngle())
        self.setWheelLinearVelocityTarget(optimizedState.speed)
        self.setSwerveAngleTarget(optimizedState.angle)


class PWMSwerveModule(SwerveModule):
    """
    Implementation of SwerveModule designed for ease of simulation:
        wheelMotor: 1:1 gearing with wheel
        swerveMotor: 1:1 gearing with swerve
        wheelEncoder: wheel distance (meters)
        swerveEncoder: swerve angle (radians)
    """

    def __init__(
        self,
        wheelMotor: PWMVictorSPX,
        swerveMotor: PWMVictorSPX,
        wheelEncoder: Encoder,
        swerveEncoder: Encoder,
    ) -> None:
        self.wheelMotor = wheelMotor
        self.swerveMotor = swerveMotor
        self.wheelEncoder = wheelEncoder
        self.swerveEncoder = swerveEncoder

        self.wheelEncoder.setDistancePerPulse(1 / constants.kWheelEncoderPulsesPerMeter)
        self.swerveEncoder.setDistancePerPulse(
            1 / constants.kSwerveEncoderPulsesPerRadian
        )

    def getSwerveAngle(self) -> Rotation2d:
        return Rotation2d(self.swerveEncoder.getDistance())

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        swerveError = swerveAngleTarget.radians() - self.swerveEncoder.getDistance()
        swerveErrorClamped = min(max(swerveError, -1), 1)
        self.swerveMotor.setSpeed(swerveErrorClamped)

    def getWheelLinearVelocity(self) -> float:
        return self.wheelEncoder.getRate()

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        speedFactor = wheelLinearVelocityTarget / constants.kMaxWheelLinearVelocity
        speedFactorClamped = min(max(speedFactor, -1), 1)
        self.wheelMotor.setSpeed(speedFactorClamped)


class CTRESwerveModule(SwerveModule):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Falcon 500 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    def __init__(
        self,
        driveMotor: WPI_TalonFX,
        steerMotor: WPI_TalonFX,
        swerveEncoder: CANCoder,
    ) -> None:
        self.driveMotor = driveMotor
        self.steerMotor = steerMotor
        self.swerveEncoder = swerveEncoder

        errorCode = self.driveMotor.config_kP(0, 0.25, 1000)
        if errorCode != ErrorCode.OK:
            print(
                "Drive Motor[{}]:config_kP: {}".format(
                    self.driveMotor.getDeviceID(), errorCode
                )
            )

    def getSwerveAngle(self) -> Rotation2d:
        steerEncoderPulses = self.steerMotor.getSelectedSensorPosition()
        swerveAngle = steerEncoderPulses / constants.kSwerveEncoderPulsesPerRadian
        print("Steer[{}]: {}".format(self.steerMotor.getDeviceID(), swerveAngle))
        return Rotation2d(swerveAngle)

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderPulsesTarget = (
            swerveAngleTarget.radians() * constants.kSwerveEncoderPulsesPerRadian
        )
        self.steerMotor.set(ControlMode.Position, steerEncoderPulsesTarget)

    def getWheelLinearVelocity(self) -> float:
        driveEncoderPulsesPerSecond = (
            self.driveMotor.getSelectedSensorVelocity()
            * constants.k100MillisecondsPerSecond
        )
        wheelLinearVelocity = (
            driveEncoderPulsesPerSecond / constants.kWheelEncoderPulsesPerMeter
        )
        return wheelLinearVelocity

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderPulsesPerSecond = (
            wheelLinearVelocityTarget * constants.kWheelEncoderPulsesPerMeter
        )
        self.driveMotor.set(
            ControlMode.Velocity,
            driveEncoderPulsesPerSecond / constants.k100MillisecondsPerSecond,
        )


class DriveSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        if RobotBase.isReal():
            self.frontLeftModule = CTRESwerveModule(
                WPI_TalonFX(constants.kFrontLeftDriveMotorId),
                WPI_TalonFX(constants.kFrontLeftSteerMotorId),
                CANCoder(constants.kFrontLeftSteerEncoderId),
            )
            self.frontRightModule = CTRESwerveModule(
                WPI_TalonFX(constants.kFrontRightDriveMotorId),
                WPI_TalonFX(constants.kFrontRightSteerMotorId),
                CANCoder(constants.kFrontRightSteerEncoderId),
            )
            self.backLeftModule = CTRESwerveModule(
                WPI_TalonFX(constants.kBackLeftDriveMotorId),
                WPI_TalonFX(constants.kBackLeftSteerMotorId),
                CANCoder(constants.kBackLeftSteerEncoderId),
            )
            self.backRightModule = CTRESwerveModule(
                WPI_TalonFX(constants.kBackRightDriveMotorId),
                WPI_TalonFX(constants.kBackRightSteerMotorId),
                CANCoder(constants.kBackRightSteerEncoderId),
            )
        else:
            self.frontLeftModule = PWMSwerveModule(
                PWMVictorSPX(constants.kFrontLeftDriveMotorSimPort),
                PWMVictorSPX(constants.kFrontLeftSteerMotorSimPort),
                Encoder(*constants.kFrontLeftDriveEncoderSimPorts),
                Encoder(*constants.kFrontLeftSteerEncoderSimPorts),
            )
            self.frontRightModule = PWMSwerveModule(
                PWMVictorSPX(constants.kFrontRightDriveMotorSimPort),
                PWMVictorSPX(constants.kFrontRightSteerMotorSimPort),
                Encoder(*constants.kFrontRightDriveEncoderSimPorts),
                Encoder(*constants.kFrontRightSteerEncoderSimPorts),
            )
            self.backLeftModule = PWMSwerveModule(
                PWMVictorSPX(constants.kBackLeftDriveMotorSimPort),
                PWMVictorSPX(constants.kBackLeftSteerMotorSimPort),
                Encoder(*constants.kBackLeftDriveEncoderSimPorts),
                Encoder(*constants.kBackLeftSteerEncoderSimPorts),
            )
            self.backRightModule = PWMSwerveModule(
                PWMVictorSPX(constants.kBackRightDriveMotorSimPort),
                PWMVictorSPX(constants.kBackRightSteerMotorSimPort),
                Encoder(*constants.kBackRightDriveEncoderSimPorts),
                Encoder(*constants.kBackRightSteerEncoderSimPorts),
            )

        self.kinematics = SwerveDrive4Kinematics(
            constants.kFrontLeftWheelPosition,
            constants.kFrontRightWheelPosition,
            constants.kBackLeftWheelPosition,
            constants.kBackRightWheelPosition,
        )

        # Create the gyro, a sensor which can indicate the heading of the robot relative
        # to a customizable position.
        self.gyro = AHRS.create_spi()

        # Create the an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = SwerveDrive4Odometry(self.kinematics, self.gyro.getRotation2d())

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.backLeftModule.getState(),
            self.backRightModule.getState(),
        )

        # rX = self.odometry.getPose().translation().X()
        # rY = self.odometry.getPose().translation().Y()
        # rAngle = int(self.odometry.getPose().rotation().degrees())

        # flAngle = int(
        #     (self.frontLeftModule.steerEncoder.getDistance() * units.radians)
        #     .to(units.degrees)
        #     .magnitude
        # )
        # frAngle = int(
        #     (self.frontRightModule.steerEncoder.getDistance() * units.radians)
        #     .to(units.degrees)
        #     .magnitude
        # )
        # blAngle = int(
        #     (self.backLeftModule.steerEncoder.getDistance() * units.radians)
        #     .to(units.degrees)
        #     .magnitude
        # )
        # brAngle = int(
        #     (self.backRightModule.steerEncoder.getDistance() * units.radians)
        #     .to(units.degrees)
        #     .magnitude
        # )

        # flSpeed = self.frontLeftModule.driveMotor.getSpeed()
        # frSpeed = self.frontRightModule.driveMotor.getSpeed()
        # blSpeed = self.backLeftModule.driveMotor.getSpeed()
        # brSpeed = self.backRightModule.driveMotor.getSpeed()

        # print(
        #     "r: {:.1f}, {:.1f}, {}* fl: {}* {:.1f} fr: {}* {:.1f} bl: {}* {:.1f} br: {}* {:.1f}".format(
        #         rX,
        #         rY,
        #         rAngle,
        #         flAngle,
        #         flSpeed,
        #         frAngle,
        #         frSpeed,
        #         blAngle,
        #         blSpeed,
        #         brAngle,
        #         brSpeed,
        #     )
        # )

    def arcadeDriveWithFactors(
        self,
        forwardSpeedFactor: float,
        sidewaysSpeedFactor: float,
        rotationSpeedFactor: float,
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param forwardSpeedFactor: the commanded forward movement
        :param sidewaysSpeedFactor: the commanded sideways movement
        :param rotationSpeedFactor: the commanded rotation
        """
        # print(
        #     "inputs: x: {:.2f} y: {:.2f} *: {:.2f}".format(
        #         forwardSpeedFactor, sidewaysSpeedFactor, rotationSpeedFactor
        #     )
        # )
        chassisSpeeds = ChassisSpeeds(
            forwardSpeedFactor * constants.kMaxForwardLinearVelocity,
            sidewaysSpeedFactor * constants.kMaxSidewaysLinearVelocity,
            rotationSpeedFactor * constants.kMaxRotationAngularVelocity,
        )

        self.arcadeDriveWithSpeeds(chassisSpeeds)

    def arcadeDriveWithSpeeds(self, chassisSpeeds: ChassisSpeeds) -> None:
        moduleStates = self.kinematics.toSwerveModuleStates(chassisSpeeds)
        (
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState,
        ) = SwerveDrive4Kinematics.normalizeWheelSpeeds(
            moduleStates, constants.kMaxWheelLinearVelocity
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)
