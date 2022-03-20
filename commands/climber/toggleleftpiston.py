from commands2 import CommandBase
from wpilib import DoubleSolenoid
from subsystems.climbers.leftclimbersubsystem import LeftClimber


class ToggleLeftPiston(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        if self.climber.leftClimb.piston.get() == DoubleSolenoid.Value.kForward:
            self.climber.leftClimb.retractPiston()
        elif self.climber.leftClimb.piston.get() == DoubleSolenoid.Value.kReverse:
            self.climber.leftClimb.extendPiston()
        elif self.climber.leftClimb.piston.get() == DoubleSolenoid.Value.kOff:
            self.climber.leftClimb.extendPiston()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
