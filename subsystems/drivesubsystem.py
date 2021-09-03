import commands2
import wpilib
import wpilib.drive
import wpimath.kinematics

import constants


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.frontLeftMotor = wpilib.PWMVictorSPX(
            constants.kFrontLeftMotorPort)
        self.backLeftMotor = wpilib.PWMVictorSPX(
            constants.kBackLeftMotorPort)
        self.frontRightMotor = wpilib.PWMVictorSPX(
            constants.kFrontRightMotorPort)
        self.backRightMotor = wpilib.PWMVictorSPX(
            constants.kBackRightMotorPort)

        # The robot's drive
        self.drive = wpilib.drive.DifferentialDrive(
            wpilib.SpeedControllerGroup(
                self.frontLeftMotor, self.backLeftMotor),
            wpilib.SpeedControllerGroup(
                self.frontRightMotor, self.backRightMotor),
        )

        # The left-side drive encoder
        self.leftEncoder = wpilib.Encoder(
            *constants.kLeftEncoderPorts,
            reverseDirection=constants.kLeftEncoderReversed
        )

        # The right-side drive encoder
        self.rightEncoder = wpilib.Encoder(
            *constants.kRightEncoderPorts,
            reverseDirection=constants.kRightEncoderReversed
        )

        # Sets the distance per pulse for the encoders
        self.leftEncoder.setDistancePerPulse(
            constants.kEncoderDistancePerPulse)
        self.rightEncoder.setDistancePerPulse(
            constants.kEncoderDistancePerPulse)

        # Create the gyro, a sensor which can indicate the heading of the robot relative
        # to a customizable position.
        self.gyro = wpilib.ADXRS450_Gyro()

        # Create the an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = wpimath.kinematics.DifferentialDriveOdometry(
            self.gyro.getRotation2d())

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.leftEncoder.getDistance(),
            self.rightEncoder.getDistance(),
        )

    def arcadeDrive(self, forwardSpeedFactor: float, rotationSpeedFactor: float) -> None:
        """
        Drives the robot using arcade controls.

        :param forwardSpeedFactor: the commanded forward movement
        :param rotationSpeedFactor: the commanded rotation
        """
        self.drive.arcadeDrive(forwardSpeedFactor, rotationSpeedFactor)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def resetOdometry(self, pose):
        """ Resets the robot's odometry to a given position."""
        self.resetEncoders()
        self.odometry.resetPosition(pose, self.gyro.getRotation2d())

    def setMaxOutput(self, maxOutputFactor: float):
        """
        Sets the max output of the drive. Useful for scaling the
        drive to drive more slowly.
        """
        self.drive.setMaxOutput(maxOutputFactor)
