from commands2 import (
    CommandBase,
    ParallelCommandGroup,
)
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber


class PivotLeftClimberToVertical(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def execute(self) -> None:
        self.climber.leftClimber.retractPiston()

    def isFinished(self) -> bool:
        return True


class PivotRightClimberToVertical(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def execute(self) -> None:
        self.climber.rightClimber.retractPiston()

    def isFinished(self) -> bool:
        return True


class PivotBothClimbersToVertical(ParallelCommandGroup):
    def __init__(self, leftClimber: LeftClimber, rightClimber: RightClimber):
        super().__init__(
            PivotLeftClimberToVertical(leftClimber),
            PivotRightClimberToVertical(rightClimber),
        )
        self.setName(__class__.__name__)
