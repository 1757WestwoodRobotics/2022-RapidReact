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

import wpilib
import wpilib.simulation
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

import constants

from pyfrc.physics.core import PhysicsInterface


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller

        # Motors
        self.frontLeftMotorSim = wpilib.simulation.PWMSim(
            constants.kFrontLeftMotorPort)
        self.frontRightMotorSim = wpilib.simulation.PWMSim(
            constants.kFrontRightMotorPort)

        self.system = LinearSystemId.identifyDrivetrainSystem(
            1.98, 0.2, 1.5, 0.3)
        self.drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self.system,
            constants.kTrackWidth,
            DCMotor.falcon500(constants.kDriveTrainMotorsPerSide),
            constants.kGearingRatio,
            constants.kWheelRadius,
        )

        self.leftEncoderSim = wpilib.simulation.EncoderSim.createForChannel(
            constants.kLeftEncoderPorts[0]
        )
        self.leftEncoderSimOffset = 0

        self.rightEncoderSim = wpilib.simulation.EncoderSim.createForChannel(
            constants.kRightEncoderPorts[0]
        )
        self.rightEncoderSimOffset = 0

        self.gyro = wpilib.simulation.ADXRS450_GyroSim(wpilib.ADXRS450_Gyro())

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Handle encoder reset
        # TODO: investigate why self.leftEncoderSim.registerResetCallback() leads to a crash when the callback is called
        if self.leftEncoderSim.getReset():
            self.leftEncoderSimOffset = self.drivesim.getLeftPosition()
        if self.rightEncoderSim.getReset():
            self.rightEncoderSimOffset = self.drivesim.getRightPosition()

        self.gyro.setAngle(-self.drivesim.getHeading().degrees())

        # Simulate the drivetrain
        frontLeftMotorSim = self.frontLeftMotorSim.getSpeed()
        frontRightMotorSim = self.frontRightMotorSim.getSpeed()

        voltage = wpilib.RobotController.getInputVoltage()
        self.drivesim.setInputs(
            frontLeftMotorSim * voltage, -frontRightMotorSim * voltage)
        self.drivesim.update(tm_diff)

        self.leftEncoderSim.setDistance(
            self.drivesim.getLeftPosition() - self.leftEncoderSimOffset)
        self.leftEncoderSim.setRate(self.drivesim.getLeftVelocity())
        self.rightEncoderSim.setDistance(
            self.drivesim.getRightPosition() - self.rightEncoderSimOffset)
        self.rightEncoderSim.setRate(self.drivesim.getRightVelocity())

        self.physics_controller.field.setRobotPose(self.drivesim.getPose())
