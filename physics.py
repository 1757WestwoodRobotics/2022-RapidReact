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
from wpilib import RobotController, SmartDashboard
from wpilib.simulation import EncoderSim, PWMSim, SimDeviceSim
import wpimath.kinematics
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d

from wpimath.system.plant import DCMotor

import constants

from pyfrc.physics.core import PhysicsInterface


class SwerveModuleSim:
    def __init__(
        self,
        position: Translation2d,
        wheelMotorSim: PWMSim,
        wheelMotorType: DCMotor,
        driveMotorGearing,
        swerveMotorSim: PWMSim,
        swerveMotorType: DCMotor,
        steerMotorGearing,
        wheelEncoderSim: EncoderSim,
        swerveEncoderSim: EncoderSim,
    ) -> None:
        self.position = position
        self.wheelMotorSim = wheelMotorSim
        self.wheelMotorType = wheelMotorType
        self.driveMotorGearing = driveMotorGearing
        self.swerveMotorSim = swerveMotorSim
        self.swerveMotorType = swerveMotorType
        self.steerMotorGearing = steerMotorGearing
        self.wheelEncoderSim = wheelEncoderSim
        self.swerveEncoderSim = swerveEncoderSim

    def __str__(self) -> str:
        return "pos: x={:.2f} y={:.2f}".format(self.position.X(), self.position.Y())


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
        deltaT = tm_diff
        states = []
        for module in self.swerveModuleSims:
            wheelVoltage = module.wheelMotorSim.getSpeed() * robotVoltage
            wheelAngularVelocity = (
                wheelVoltage
                * module.wheelMotorType.Kv
                / module.driveMotorGearing  # scale the wheel motor to get more reasonable wheel speeds
            )
            wheelLinearVelocity = (
                wheelAngularVelocity * constants.kWheelDistancePerRadian
            )
            module.wheelEncoderSim.setRate(wheelLinearVelocity)

            deltaWheelDistance = wheelLinearVelocity * deltaT
            newWheelDistance = module.wheelEncoderSim.getDistance() + deltaWheelDistance
            module.wheelEncoderSim.setDistance(newWheelDistance)

            swerveVoltage = module.swerveMotorSim.getSpeed() * robotVoltage
            swerveAngularVelocity = (
                swerveVoltage
                * module.swerveMotorType.Kv
                / module.steerMotorGearing  # scale the swerve motor to get more reasonable swerve speeds
            )
            module.swerveEncoderSim.setRate(swerveAngularVelocity)

            deltaSwerveAngle = swerveAngularVelocity * deltaT
            newSwerveAngle = module.swerveEncoderSim.getDistance() + deltaSwerveAngle
            module.swerveEncoderSim.setDistance(newSwerveAngle)

            state = wpimath.kinematics.SwerveModuleState(
                wheelLinearVelocity,
                Rotation2d(newSwerveAngle),
            )
            states.append(state)

        chassisSpeed = self.kinematics.toChassisSpeeds(states)
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

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
            PWMSim(constants.kSimFrontLeftDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimFrontLeftSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimFrontLeftDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimFrontLeftSteerEncoderPorts[0]),
        )
        self.frontRightModuleSim = SwerveModuleSim(
            constants.kFrontRightWheelPosition,
            PWMSim(constants.kSimFrontRightDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimFrontRightSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimFrontRightDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimFrontRightSteerEncoderPorts[0]),
        )
        self.backSimLeftModule = SwerveModuleSim(
            constants.kBackLeftWheelPosition,
            PWMSim(constants.kSimBackLeftDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimBackLeftSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimBackLeftDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimBackLeftSteerEncoderPorts[0]),
        )
        self.backSimRightModule = SwerveModuleSim(
            constants.kBackRightWheelPosition,
            PWMSim(constants.kSimBackRightDriveMotorPort),
            DCMotor.falcon500(),
            constants.kDriveGearingRatio,
            PWMSim(constants.kSimBackRightSteerMotorPort),
            DCMotor.falcon500(),
            constants.kSteerGearingRatio,
            EncoderSim.createForChannel(constants.kSimBackRightDriveEncoderPorts[0]),
            EncoderSim.createForChannel(constants.kSimBackRightSteerEncoderPorts[0]),
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backSimLeftModule,
            self.backSimRightModule,
        ]

        self.driveSim = SwerveDriveSim(tuple(self.swerveModuleSims))

        self.gyroSim = SimDeviceSim("navX-Sensor[4]")
        self.gyroYaw = self.gyroSim.getDouble("Yaw")

        simTargetObject = self.physics_controller.field.getObject(
            constants.kSimTargetName
        )
        simTargetObject.setPose(constants.kSimDefaultTargetLocation)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        self.gyroYaw.set(-self.driveSim.getHeading().degrees())

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.driveSim.update(tm_diff, voltage)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)
        SmartDashboard.putNumberArray(
            constants.kSimRobotPoseArrayKey,
            [simRobotPose.X(), simRobotPose.Y(), simRobotPose.rotation().radians()],
        )

        simTargetObject = self.physics_controller.field.getObject(
            constants.kSimTargetName
        )
        simTargetPose = simTargetObject.getPose()
        SmartDashboard.putNumberArray(
            constants.kSimTargetPoseArrayKey,
            [simTargetPose.X(), simTargetPose.Y(), simTargetPose.rotation().radians()],
        )

        if SmartDashboard.getBoolean(constants.kTargetPoseArrayKeys.validKey, False):
            targetPoseX, targetPoseY, targetAngle = SmartDashboard.getNumberArray(
                constants.kTargetPoseArrayKeys.valueKey, [0, 0, 0]
            )
            targetPose = Pose2d(targetPoseX, targetPoseY, targetAngle)
            targetObject = self.physics_controller.field.getObject(
                constants.kTargetName
            )
            targetObject.setPose(targetPose)
