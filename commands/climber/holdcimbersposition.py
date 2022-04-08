from commands2 import CommandBase, ParallelCommandGroup
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber


class HoldLeftClimberPosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.leftClimber.climberMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()


class HoldRightClimberPosition(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.rightClimber.climberMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()


class HoldBothClimbersPosition(ParallelCommandGroup):
    def __init__(self, leftClimber: LeftClimber, rightClimber: RightClimber):
        super().__init__(
            HoldLeftClimberPosition(leftClimber),
            HoldRightClimberPosition(rightClimber),
        )
        self.setName(__class__.__name__)
