from commands2 import CommandBase
from wpilib import RobotBase
from subsystems.climbers.rightclimbersubsystem import RightClimber


class HoldRightClimberExtension(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        if RobotBase.isReal():
            self.climber.rightClimb.climbMotor.motor.neutralOutput()
        else:
            self.climber.rightClimb.climbMotor.setSpeed(0)
        self.climber.rightClimb.activateBrake()
