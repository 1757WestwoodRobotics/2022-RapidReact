from commands2 import SubsystemBase
from wpilib import SmartDashboard
from subsystems.climbers.climbersubsystem import ClimberModule
import constants


class RightClimber(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.rightClimber = ClimberModule(
            constants.kRightClimberMotorName,
            constants.kRightClimberMotorCanID,
            constants.kClimberMotorPGain,
            constants.kClimberMotorIGain,
            constants.kClimberMotorDGain,
            constants.kRightClimberBrakePCMID,
            constants.kRightClimberPivotSolenoidForwardActuationID,
            constants.kRightClimberPivotSolenoidBackwardActuationID,
        )

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            constants.kRightClimberEncoderTicksKey,
            self.rightClimber.climberMotor.getSelectedSensorPosition(),
        )
