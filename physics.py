#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import typing
import wpilib
import wpilib.simulation
import wpimath.kinematics
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d

from wpimath.system.plant import DCMotor

import constants

from pyfrc.physics.core import PhysicsInterface
from units import units


class SwerveDriveSimOutput:
    def __init__(
        self,
        driveEncoderPosition: float,
        driveEncoderVelocity: float,
        steerEncoderPosition: float,
        steerEncoderVelocity: float,
    ) -> None:
        self.driveEncoderPosition = driveEncoderPosition
        self.driveEncoderVelocity = driveEncoderVelocity
        self.steerEncoderPosition = steerEncoderPosition
        self.steerEncoderVelocity = steerEncoderVelocity


class SwerveModuleSim:
    def __init__(
        self,
        position: Translation2d,
        driveMotorSim,
        driveMotorType: DCMotor,
        driveMotorGearing,
        steerMotorSim,
        steerMotorType: DCMotor,
        steerMotorGearing,
        driveEncoderSim,
        steerEncoderSim,
    ) -> None:
        self.position = position
        self.driveMotorSim = driveMotorSim
        self.driveMotorType = driveMotorType
        self.driveMotorGearing = driveMotorGearing
        self.steerMotorSim = steerMotorSim
        self.steerMotorType = steerMotorType
        self.steerMotorGearing = steerMotorGearing
        self.driveEncoderSim = driveEncoderSim
        self.steerEncoderSim = steerEncoderSim

    def __str__(self) -> str:
        return "pos: x={:.2f} y={:.2f}".format(self.position.X(), self.position.Y())

    def applyOutput(self, swerveDriveSimOutput: SwerveDriveSimOutput):
        self.driveEncoderSim.setDistance(swerveDriveSimOutput.driveEncoderPosition)
        self.driveEncoderSim.setRate(swerveDriveSimOutput.driveEncoderVelocity)
        self.steerEncoderSim.setDistance(swerveDriveSimOutput.steerEncoderPosition)
        self.steerEncoderSim.setRate(swerveDriveSimOutput.steerEncoderVelocity)


class SwerveDriveSim:
    def __init__(self, swerveModuleSims: typing.Tuple[SwerveModuleSim, ...]) -> None:
        self.swerveModuleSims = swerveModuleSims
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            *(module.position for module in swerveModuleSims)
        )
        self.pose = Pose2d()
        self.outputs = None

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, robotVoltage: float) -> None:
        deltaT = tm_diff * units.seconds
        states = []
        for module in self.swerveModuleSims:
            driveVoltage = module.driveMotorSim.getSpeed() * robotVoltage
            driveAngularSpeed = (
                driveVoltage * module.driveMotorType.Kv * (units.radians / units.second)
            )
            wheelAngularSpeed = driveAngularSpeed / constants.kDriveGearingRatio
            wheelSpeed = wheelAngularSpeed * constants.kWheelDistancePerRevolution
            module.driveEncoderSim.setRate(
                wheelSpeed.to(units.meters / units.second).magnitude
            )

            deltaWheelDistance = wheelSpeed * deltaT
            newWheelDistance = (
                module.driveEncoderSim.getDistance() * units.meters + deltaWheelDistance
            )
            module.driveEncoderSim.setDistance(
                newWheelDistance.to(units.meters).magnitude
            )

            steerVoltage = module.steerMotorSim.getSpeed() * robotVoltage
            steerMotorAngularSpeed = (
                steerVoltage * module.steerMotorType.Kv * (units.radians / units.second)
            )
            steerAngularSpeed = steerMotorAngularSpeed / constants.kSteerGearingRatio
            module.steerEncoderSim.setRate(
                steerAngularSpeed.to(units.radians / units.seconds).magnitude
            )
            deltaSteerAngle = steerAngularSpeed * deltaT
            newSteerAngle = (
                module.steerEncoderSim.getDistance() * units.radians + deltaSteerAngle
            )
            module.steerEncoderSim.setDistance(
                newSteerAngle.to(units.radians).magnitude
            )

            state = wpimath.kinematics.SwerveModuleState(
                module.driveEncoderSim.getRate(),
                Rotation2d(module.steerEncoderSim.getDistance()),
            )
            states.append(state)

        chassisSpeed = self.kinematics.toChassisSpeeds(states)
        deltaHeading = chassisSpeed.omega * (units.radians / units.second) * deltaT
        deltaX = chassisSpeed.vx * (units.meters / units.second) * deltaT
        deltaY = chassisSpeed.vy * (units.meters / units.second) * deltaT

        deltaTrans = Transform2d(
            deltaX.to(units.meters).magnitude,
            deltaY.to(units.meters).magnitude,
            deltaHeading.to(units.radians).magnitude,
        )

        newPose = self.pose + deltaTrans
        self.pose = newPose


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller

        self.frontLeftModuleSim = SwerveModuleSim(
            constants.kFrontLeftWheelPosition,
            wpilib.simulation.PWMSim(constants.kFrontLeftDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            wpilib.simulation.PWMSim(constants.kFrontLeftSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kFrontLeftDriveEncoderPorts[0]
            ),
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kFrontLeftSteerEncoderPorts[0]
            ),
        )
        self.frontRightModuleSim = SwerveModuleSim(
            constants.kFrontRightWheelPosition,
            wpilib.simulation.PWMSim(constants.kFrontRightDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            wpilib.simulation.PWMSim(constants.kFrontRightSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kFrontRightDriveEncoderPorts[0]
            ),
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kFrontRightSteerEncoderPorts[0]
            ),
        )
        self.backLeftModuleSim = SwerveModuleSim(
            constants.kBackLeftWheelPosition,
            wpilib.simulation.PWMSim(constants.kBackLeftDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            wpilib.simulation.PWMSim(constants.kBackLeftSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kBackLeftDriveEncoderPorts[0]
            ),
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kBackLeftSteerEncoderPorts[0]
            ),
        )
        self.backRightModuleSim = SwerveModuleSim(
            constants.kBackRightWheelPosition,
            wpilib.simulation.PWMSim(constants.kBackRightDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            wpilib.simulation.PWMSim(constants.kBackRightSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kBackRightDriveEncoderPorts[0]
            ),
            wpilib.simulation.EncoderSim.createForChannel(
                constants.kBackRightSteerEncoderPorts[0]
            ),
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backLeftModuleSim,
            self.backRightModuleSim,
        ]

        self.drivesim = SwerveDriveSim(tuple(self.swerveModuleSims))

        self.gyro = wpilib.simulation.ADXRS450_GyroSim(wpilib.ADXRS450_Gyro())

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        self.gyro.setAngle(-self.drivesim.getHeading().degrees())

        # Simulate the drivetrain
        voltage = wpilib.RobotController.getInputVoltage()

        self.drivesim.update(tm_diff, voltage)

        self.physics_controller.field.setRobotPose(self.drivesim.getPose())
