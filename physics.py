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
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.system.plant import DCMotor
import wpimath.kinematics
from networktables import NetworkTables
from pyfrc.physics.core import PhysicsInterface

import constants
from util import convenientmath


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
        return f"pos: x={self.position.X():.2f} y={self.position.Y():.2f}"


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


class LimelightSim:
    def __init__(self) -> None:
        NetworkTables.initialize()
        self.limelightNetworkTable = NetworkTables.getTable(
            constants.kLimelightNetworkTableName
        )

    def update(self, limelightPose: Pose2d, targetPose: Pose2d) -> None:
        targetInLimelight = Transform2d(limelightPose, targetPose)
        targetAngle = convenientmath.rotationFromTranslation(
            targetInLimelight.translation()
        )
        targetValid = constants.kLimelightTargetInvalidValue
        if (
            constants.kLimelightMinHorizontalFoV.radians()
            < targetAngle.radians()
            < constants.kLimelightMaxHorizontalFoV.radians()
        ):
            targetValid = constants.kLimelightTargetValidValue
            self.limelightNetworkTable.putNumber(
                constants.kLimelightTargetHorizontalAngleKey,
                -1 * targetAngle.degrees(),  # limelight uses reversed direction along x
            )

        self.limelightNetworkTable.putNumber(
            constants.kLimelightTargetValidKey, targetValid
        )


class IntakeCameraSim:
    def __init__(self) -> None:
        NetworkTables.initialize()
        self.photonvisionNetworkTable = NetworkTables.getTable(
            constants.kPhotonvisionNetworkTableName
        )

    def update(self, intakeCameraPose: Pose2d, ballPose: Pose2d) -> None:
        ballInCamera = Transform2d(intakeCameraPose, ballPose)
        ballAngle = convenientmath.rotationFromTranslation(ballInCamera.translation())
        ballValid = False
        if (
            constants.kIntakeCameraMinHorizontalFOV.radians()
            < ballAngle.radians()
            < constants.kIntakeCameraMaxHorizontalFOV.radians()
        ):
            ballValid = True
            ballDistance = intakeCameraPose.translation().distance(
                ballPose.translation()
            )
            self.photonvisionNetworkTable.putNumber(
                constants.kPhotonvisionTargetSimDistanceKey, ballDistance
            )
            self.photonvisionNetworkTable.putNumber(
                constants.kPhotonvisionTargetHorizontalAngleKey, ballAngle.degrees()
            )

        self.photonvisionNetworkTable.putBoolean(
            constants.kPhotonvisionTargetValidKey, ballValid
        )


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

        simBallObject = self.physics_controller.field.getObject(constants.kSimBallName)
        simBallObject.setPose(constants.kSimDefaultBallLocation)

        self.limelightSim = LimelightSim()
        self.intakeCameraSim = IntakeCameraSim()

    # pylint: disable=unused-argument
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

        # publish the simulated robot pose to nt
        SmartDashboard.putNumberArray(
            constants.kSimRobotPoseArrayKey,
            [simRobotPose.X(), simRobotPose.Y(), simRobotPose.rotation().radians()],
        )

        # publish the simulated target and ball pose to nt
        simTargetObject = self.physics_controller.field.getObject(
            constants.kSimTargetName
        )
        simTargetPose = simTargetObject.getPose()
        SmartDashboard.putNumberArray(
            constants.kSimTargetPoseArrayKey,
            [simTargetPose.X(), simTargetPose.Y(), simTargetPose.rotation().radians()],
        )

        simBallObject = self.physics_controller.field.getObject(constants.kSimBallName)
        simBallPose = simBallObject.getPose()
        SmartDashboard.putNumberArray(
            constants.kSimBallPoseArrayKey,
            [simBallPose.X(), simBallPose.Y(), simBallPose.rotation().radians()],
        )

        simIntakeCameraPose = simRobotPose.transformBy(
            Transform2d(
                Translation2d(constants.kIntakeCameraCenterOffsetInMeters, 0.0),
                Rotation2d(),
            )
        )
        simIntakeCameraObject = self.physics_controller.field.getObject(
            constants.kSimIntakeCameraObjectName
        )
        simIntakeCameraObject.setPose(simIntakeCameraPose)

        self.intakeCameraSim.update(simIntakeCameraPose, simBallPose)
        # publish the simulated limelight nt entries
        limelightPanAngle = SmartDashboard.getNumber(constants.kTrackerPanAngleKey, 0)
        robotToLimelightTransform = Transform2d(
            constants.kLimelightMountingOffset, Rotation2d(limelightPanAngle)
        )

        simLimelightPose = simRobotPose + robotToLimelightTransform

        servoPose = Pose2d(
            simLimelightPose.translation(),
            simLimelightPose.rotation()
            + Rotation2d.fromDegrees(
                SmartDashboard.getNumber(constants.kCameraServoRotationNumberKey, 0.0)
            ),
        )
        self.limelightSim.update(servoPose, simTargetPose)

        servoObject = self.physics_controller.field.getObject(
            constants.kCameraSimServoObjectName
        )
        servoObject.setPose(servoPose)

        # show the robot's estimation of where the target is on the simulated field
        if SmartDashboard.getBoolean(constants.kTargetPoseArrayKeys.validKey, False):
            targetPoseX, targetPoseY, targetAngle = SmartDashboard.getNumberArray(
                constants.kTargetPoseArrayKeys.valueKey, [0, 0, 0]
            )
            targetPose = Pose2d(targetPoseX, targetPoseY, targetAngle)
            targetObject = self.physics_controller.field.getObject(
                constants.kTargetName
            )
            targetObject.setPose(targetPose)
