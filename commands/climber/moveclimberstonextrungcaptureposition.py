from commands2 import CommandBase
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class MoveLeftClimberToNextRungCapturePosition(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.leftClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimber.extendPiston()
        self.climber.leftClimber.setClimberTiltedExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.leftClimber.climberMotor.neutralOutput()
        self.climber.leftClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.leftClimber.climberMotor.getSelectedSensorPosition()
                    - constants.kClimberTiltedExtensionMax
                )
            )
            < constants.kClimberExtensionPositionThreshold
        )


class MoveRightClimberToNextRungCapturePosition(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements([self.climber])

    def initialize(self) -> None:
        self.climber.rightClimber.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimber.extendPiston()
        self.climber.rightClimber.setClimberTiltedExtension()

    def end(self, _interrupted: bool) -> None:
        self.climber.rightClimber.climberMotor.neutralOutput()
        self.climber.rightClimber.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.rightClimber.climberMotor.getSelectedSensorPosition()
                    - constants.kClimberTiltedExtensionMax
                )
            )
            < constants.kClimberExtensionPositionThreshold
        )
