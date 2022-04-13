from commands2 import (
    CommandBase,
    ParallelCommandGroup,
    SequentialCommandGroup,
    WaitCommand,
)
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class MoveLeftClimberToMiddleRungCapturePosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.leftClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimber.retractPiston()
        self.climber.leftClimber.setClimberFullExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.leftClimber.climberMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.leftClimber.climberMotor.getSelectedSensorPosition()
                    - constants.kClimberMiddleRungCapturePosition
                )
            )
            < constants.kClimberExtensionPositionThreshold
        )


class MoveRightClimberToMiddleRungCapturePosition(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.rightClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimber.retractPiston()
        self.climber.rightClimber.setClimberFullExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.rightClimber.climberMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.rightClimber.climberMotor.getSelectedSensorPosition()
                    - constants.kClimberMiddleRungCapturePosition
                )
            )
            < constants.kClimberExtensionPositionThreshold
        )


class MoveBothClimbersToMiddleRungCapturePositionMovements(ParallelCommandGroup):
    def __init__(self, leftClimber: LeftClimber, rightClimber: RightClimber):
        super().__init__(
            MoveLeftClimberToMiddleRungCapturePosition(leftClimber),
            MoveRightClimberToMiddleRungCapturePosition(rightClimber),
        )
        self.setName(__class__.__name__)


class MoveBothClimbersToMiddleRungCapturePosition(SequentialCommandGroup):
    def __init__(self, leftclimber: LeftClimber, rightclimber: RightClimber):
        super().__init__(
            WaitCommand(constants.kClimberPauseBeforeMovement),
            MoveBothClimbersToMiddleRungCapturePositionMovements(
                leftclimber, rightclimber
            ),
        )
