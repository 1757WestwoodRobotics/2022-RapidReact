from commands2 import CommandBase
from wpilib import RobotBase
from subsystems.climbers.leftclimbersubsystem import LeftClimber


class HoldLeftClimberExtension(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        if RobotBase.isReal():
            self.climber.leftClimb.climbMotor.motor.neutralOutput()
        else:
            self.climber.leftClimb.climbMotor.setSpeed(0)
        self.climber.leftClimb.activateBrake()
