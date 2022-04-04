from commands1 import WaitCommand
from commands2 import CommandBase, SequentialCommandGroup
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class LeftClimberPivotVertical(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimber.retractPiston()

    def isFinished(self) -> bool:
        return True


class RetractLeftClimberToFullHangingPosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimber.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.leftClimber.climberMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.leftClimber.climberMotor.getSelectedSensorPosition()
                    - constants.kClimberHangingPosition
                )
            )
            < constants.kClimberRetractionPositionThreshold
        )


class RightClimberPivotVertical(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimber.retractPiston()

    def isFinished(self) -> bool:
        return True


class MoveRightClimberToFullHangingPosition(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimber.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.rightClimber.climberMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.rightClimber.climberMotor.getSelectedSensorPosition()
                    - constants.kClimberHangingPosition
                )
            )
            < constants.kClimberRetractionPositionThreshold
        )


class MoveLeftClimberToFullHangingPosition(SequentialCommandGroup):
    def __init__(self, climber: LeftClimber):
        super().__init__(
            LeftClimberPivotVertical(climber),
            WaitCommand(2),
            RetractLeftClimberToFullHangingPosition(climber),
        )
        self.setName(__class__.__name__)
