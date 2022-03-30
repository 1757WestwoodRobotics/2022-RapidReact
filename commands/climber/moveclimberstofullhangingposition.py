from commands2 import CommandBase
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class MoveLeftClimberToFullHangingPosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimber.retractPiston()
        self.climber.leftClimber.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.leftClimber.climbMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.leftClimber.climbMotor.getPosition()
                    - constants.kClimberHangingPosition
                )
            )
            < constants.kClimberRetractionPositionThreshold
        )


class MoveRightClimberToFullHangingPosition(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimber.retractPiston()
        self.climber.rightClimber.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.rightClimber.climbMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.rightClimber.climbMotor.getPosition()
                    - constants.kClimberHangingPosition
                )
            )
            < constants.kClimberRetractionPositionThreshold
        )
