from commands2 import CommandBase
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber


class PivotLeftPistonToTilted(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimber.extendPiston()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True


class PivotRightPistonToTilted(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimber.extendPiston()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True


class PivotLeftPistonToVertical(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimber.retractPiston()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True


class PivotRightPistonToVertical(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimber.retractPiston()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True
