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
    def runIndexer(self) -> None:
        self.indexerRunning = True
        if self.indexerReversed:
            self.indexerMotor.setSpeed(-1 * constants.kIndexerSpeed)
        else:
            self.indexerMotor.setSpeed(constants.kIndexerSpeed)

    def stopIndexer(self) -> None:
        self.indexerRunning = False
        self.indexerMotor.setSpeed(0)

    def runStaging(self) -> None:
        if self.stagingReversed:
            self.stagingMotor.setSpeed(-1 * constants.kStagingSpeed)
        else:
            self.stagingMotor.setSpeed(constants.kStagingSpeed)
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
            self.runIndexer()
            self.runStaging()
