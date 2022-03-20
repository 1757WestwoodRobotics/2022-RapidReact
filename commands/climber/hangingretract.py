from commands2 import CommandBase
from wpilib import RobotBase
from subsystems.climbers.leftclimbersubsystem import LeftClimber
from subsystems.climbers.rightclimbersubsystem import RightClimber
import constants


class HangingLeftClimber(CommandBase):
    def __init__(self, climber: LeftClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.leftClimb.deactivateBrake()

    def execute(self) -> None:
        self.climber.leftClimb.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        if RobotBase.isReal():
            self.climber.leftClimb.climbMotor.motor.neutralOutput()
        else:
            self.climber.leftClimb.climbMotor.setSpeed(0)
        self.climber.leftClimb.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.leftClimb.climbMotor.getPosition()
                    - constants.kClimbHangingExtension
                )
            )
            < constants.kClimbRetractionMotorThreshold
        )


class HangingRightClimber(CommandBase):
    def __init__(self, climber: RightClimber) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climber
        self.addRequirements(climber)

    def initialize(self) -> None:
        self.climber.rightClimb.deactivateBrake()

    def execute(self) -> None:
        self.climber.rightClimb.setClimberHangingExtension()

    def end(self, _interrupted: bool) -> None:
        if RobotBase.isReal():
            self.climber.rightClimb.climbMotor.motor.neutralOutput()
        else:
            self.climber.rightClimb.climbMotor.setSpeed(0)
        self.climber.rightClimb.activateBrake()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return (
            abs(
                (
                    self.climber.rightClimb.climbMotor.getPosition()
                    - constants.kClimbHangingExtension
                )
            )
            < constants.kClimbRetractionMotorThreshold
        )
