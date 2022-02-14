from commands2 import SubsystemBase
import constants
from util.helpfulIO import Falcon


class IndexerSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexerRunning = False
        self.indexerReversed = False
        self.indexerMotor = Falcon(
            constants.kIndexerMotorName,
            constants.kIndexerMotorId,
            constants.kSimIndexerMotorPort,
        )
        self.stagingRunning = False
        self.stagingReversed = True  # We want balls to stay in when collected
        self.stagingMotor = Falcon(
            constants.kStagingMotorName,
            constants.kStagingMotorId,
            constants.kSimStagingMotorPort,
        )

    # Checks are accessible outside of the subsystem
    def isIndexerRunning(self) -> bool:
        return self.indexerRunning

    def isIndexerReversed(self) -> bool:
        return self.indexerReversed

    def isStagingRunning(self) -> bool:
        return self.stagingRunning

    def isStagingReversed(self) -> bool:
        return self.stagingReversed

    # Switches direction to reverse the ball path
    def toggleReverseBallPath(self) -> None:
        self.indexerReversed = not self.indexerReversed
        self.stagingReversed = False

    def reverseStaging(self) -> None:
        self.stagingReversed = True

    def regularReverseSettings(self) -> None:
        self.indexerReversed = False
        self.stagingReversed = True

    # Basic run and stop using the reverse as a boolean for direction
    def runIndexer(self, reverse: bool) -> None:
        self.indexerRunning = True
        if reverse:
            self.indexerMotor.setSpeed(-1000)
        else:
            self.indexerMotor.setSpeed(1000)

    def stopIndexer(self) -> None:
        self.indexerRunning = False
        self.indexerMotor.setSpeed(0)

    def runStaging(self, reverse: bool) -> None:
        if reverse:
            self.stagingMotor.setSpeed(-1000)
        else:
            self.stagingMotor.setSpeed(1000)
        self.stagingRunning = True

    def stopStaging(self) -> None:
        self.stagingRunning = False
        self.stagingMotor.setSpeed(0)

    def toggleIndexerSystem(self) -> None:
        if self.indexerRunning:
            self.stopIndexer()
            self.stopStaging()
        else:
            self.indexerReversed = False
            self.runIndexer(self.indexerReversed)
            self.runStaging(self.stagingReversed)