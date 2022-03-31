from commands2 import SubsystemBase
from wpilib import SmartDashboard
from subsystems.climbers.climbersubsystem import ClimberModule
import constants


class LeftClimber(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.leftClimber = ClimberModule(
            constants.kLeftClimberMotorName,
            constants.kLeftClimberMotorCanID,
            constants.kClimberMotorPGain,
            constants.kClimberMotorIGain,
            constants.kClimberMotorDGain,
            constants.kLeftClimberBrakePCMID,
            constants.kLeftClimberPivotSolenoidForwardActuationID,
            constants.kLeftClimberPivotSolenoidBackwardActuationID,
            constants.kLeftClimberInverted,
        )

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            constants.kLeftClimberEncoderTicksKey,
            self.leftClimber.climberMotor.getSelectedSensorPosition(),
        )
