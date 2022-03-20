from commands2 import SubsystemBase
from wpilib import SmartDashboard
from subsystems.climbers.climbersubsystem import ClimberModule
import constants


class LeftClimber(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.leftClimb = ClimberModule(
            constants.kLeftClimberMotorName,
            constants.kLeftClimbMotorCanID,
            constants.kSimLeftClimbMotorID,
            constants.kClimbMotorPGain,
            constants.kLeftClimbBrakePCMID,
            constants.kLeftClimbForwardActuationID,
            constants.kLeftClimbBackwardActuationID,
            constants.kLeftClimbInverted,
        )

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            constants.kLeftClimberEncoderTicksKey,
            self.leftClimb.climbMotor.getPosition(),
        )
