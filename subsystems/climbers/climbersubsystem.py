from wpilib import (
    DoubleSolenoid,
    PneumaticsModuleType,
    RobotBase,
    Solenoid,
)
from util.helpfulIO import Falcon
import constants


class ClimberModule:
    def __init__(
        self,
        motorName,
        realMotorID,
        simMotorID,
        motorPGain,
        brakeID,
        actuatorID1,
        actuatorID2,
        inverted=False,
    ) -> None:
        self.brake = Solenoid(PneumaticsModuleType.REVPH, brakeID)
        self.piston = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            actuatorID1,
            actuatorID2,
        )
        self.climbMotor = Falcon(
            motorName, realMotorID, simMotorID, motorPGain, inverted=inverted
        )
        if RobotBase.isReal():
            self.climbMotor.motor.configReverseSoftLimitThreshold(
                constants.kClimbHangingExtension
            )
            self.climbMotor.motor.configReverseSoftLimitEnable(True)
        self.extendPiston()

    def activateBrake(self) -> None:
        self.brake.set(False)

    def deactivateBrake(self) -> None:
        self.brake.set(True)

    def extendPiston(self) -> None:
        self.piston.set(DoubleSolenoid.Value.kForward)

    def retractPiston(self) -> None:
        self.piston.set(DoubleSolenoid.Value.kReverse)

    def setClimberFullExtension(self) -> None:
        self.climbMotor.setPosition(constants.kClimbFullExtension)

    def setClimberHangingExtension(self) -> None:
        self.climbMotor.setPosition(constants.kClimbHangingExtension)
