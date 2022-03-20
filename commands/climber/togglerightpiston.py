from commands2 import CommandBase
from wpilib import DoubleSolenoid
from subsystems.climbers.rightclimbersubsystem import RightClimber


class ToggleRightPiston(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        if self.climber.rightClimb.piston.get() == DoubleSolenoid.Value.kForward:
            self.climber.rightClimb.retractPiston()
        elif self.climber.rightClimb.piston.get() == DoubleSolenoid.Value.kReverse:
            self.climber.rightClimb.extendPiston()
        elif self.climber.rightClimb.piston.get() == DoubleSolenoid.Value.kOff:
            self.climber.rightClimb.extendPiston()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
