from commands2 import CommandBase, ParallelCommandGroup
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class MoveLeftClimberToMiddleRungCapturePosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimber.setClimberFullExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.leftClimber.climbMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.leftClimber.climbMotor.getPosition()
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
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimber.setClimberFullExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.rightClimber.climbMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.rightClimber.climbMotor.getPosition()
                    - constants.kClimberMiddleRungCapturePosition
                )
            )
            < constants.kClimberExtensionPositionThreshold
        )


class MoveBothClimbersToMiddleRungCapturePosition(ParallelCommandGroup):
    def __init__(self, leftClimber: LeftClimber, rightClimber: RightClimber):
        super().__init__(
            MoveLeftClimberToMiddleRungCapturePosition(leftClimber),
            MoveRightClimberToMiddleRungCapturePosition(rightClimber),
        )
        self.setName(__class__.__name__)