from commands2 import CommandBase, ParallelCommandGroup
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class MoveLeftClimberToMiddleRungHangPosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.leftClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimber.retractPiston()
        self.climber.leftClimber.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.leftClimber.climberMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()

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


class MoveRightClimberToMiddleRungHangPosition(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.rightClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimber.retractPiston()
        self.climber.rightClimber.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.rightClimber.climberMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()

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


class MoveBothClimbersToMiddleRungHangPosition(ParallelCommandGroup):
    def __init__(self, leftClimber: LeftClimber, rightClimber: RightClimber):
        super().__init__(
            MoveLeftClimberToMiddleRungHangPosition(leftClimber),
            MoveRightClimberToMiddleRungHangPosition(rightClimber),
        )
        self.setName(__class__.__name__)
