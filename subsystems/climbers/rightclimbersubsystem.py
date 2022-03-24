from commands2 import SubsystemBase
from wpilib import SmartDashboard
from subsystems.climbers.climbersubsystem import ClimberModule
import constants


class RightClimber(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.rightClimb = ClimberModule(
            constants.kRightClimberMotorName,
            constants.kRightClimberMotorCanID,
            constants.kSimRightClimberMotorID,
            constants.kClimberMotorPGain,
            constants.kRightClimberBrakePCMID,
            constants.kRightClimberForwardActuationID,
            constants.kRightClimberBackwardActuationID,
        )

    def periodic(self) -> None:
        SmartDashboard.putNumber(
            constants.kRightClimberEncoderTicksKey,
            self.rightClimb.climbMotor.getPosition(),
        )
