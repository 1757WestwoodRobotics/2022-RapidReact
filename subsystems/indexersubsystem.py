from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import SmartDashboard
import constants
from util.helpfulIO import Falcon, LimitSwitch


class IndexerSubsystem(SubsystemBase):
    class Mode(Enum):
        Holding = auto()
        FeedForward = auto()
        Reversed = auto()
        Off = auto()

        def asString(self) -> str:
            mapping = {
                self.Holding: "Holding",
                self.FeedForward: "Feed Forward",
                self.Reversed: "Reversed",
                self.Off: "Off",
            }
            return mapping[self]

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexerMotor = Falcon(
            constants.kIndexerMotorName,
            constants.kIndexerMotorId,
            constants.kSimIndexerMotorPort,
        )
        self.stagingMotor = Falcon(
            constants.kStagingMotorName,
            constants.kStagingMotorId,
            constants.kSimStagingMotorPort,
        )
        self.indexerSensor = LimitSwitch(
            self.indexerMotor,
            constants.kForwardSensorIndexer,
            constants.kSimIndexerSensorId,
        )
        self.stagingSensor = LimitSwitch(
            self.stagingMotor,
            constants.kForwardSensorStaging,
            constants.kSimStagingSensorId,
        )
        self.state = self.Mode.Holding

    def periodic(self) -> None:
        SmartDashboard.putString(
            constants.kIndexerSystemStateKey, self.state.asString()
        )

    # Switches direction to reverse the ball path
    def motorsOff(self) -> None:
        self.state = self.Mode.Off

    def feedBallForward(self) -> None:
        self.state = self.Mode.FeedForward

    def holdBall(self) -> None:
        self.state = self.Mode.Holding

    def reversePath(self) -> None:
        self.state = self.Mode.Reversed

    def defaultIndexer(self) -> None:
        if self.state == self.Mode.FeedForward:
            self.indexerMotor.setSpeed(constants.kIndexerSpeed)
            self.stagingMotor.setSpeed(constants.kStagingSpeed)
        elif self.state == self.Mode.Holding:
            if self.indexerSensor.value() and self.stagingSensor.value():
                self.indexerMotor.setSpeed(0)
                self.stagingMotor.setSpeed(0)
            else:
                self.indexerMotor.setSpeed(constants.kIndexerSpeed)
                self.stagingMotor.setSpeed(-constants.kStagingSpeed)
        elif self.state == self.Mode.Reversed:
            self.indexerMotor.setSpeed(-constants.kIndexerSpeed)
            self.stagingMotor.setSpeed(-constants.kStagingSpeed)
        elif self.state == self.Mode.Off:
            self.indexerMotor.setSpeed(0)
            self.stagingMotor.setSpeed(0)
