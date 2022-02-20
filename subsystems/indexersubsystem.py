from commands2 import SubsystemBase
import constants
from util.helpfulIO import Falcon


class IndexerSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexerRunning = True
        self.indexerReversed = False
        self.indexerMotor = Falcon(
            constants.kIndexerMotorName,
            constants.kIndexerMotorId,
            constants.kSimIndexerMotorPort,
        )
        self.stagingRunning = True
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
    def motorsOff(self) -> None:
        self.indexerRunning = False
        self.stagingRunning = False

    def runMotors(self) -> None:
        self.indexerRunning = True
        self.stagingRunning = True

    def feedBallForward(self) -> None:
        self.stagingReversed = False
        self.indexerReversed = False
        self.runMotors()

    def holdBall(self) -> None:
        self.stagingReversed = True
        self.indexerReversed = False
        self.runMotors()

    def reversePath(self) -> None:
        self.stagingReversed = True
        self.indexerReversed = True
        self.runMotors()

    def reverseIndexer(self) -> None:
        self.indexerReversed = not self.indexerReversed

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

    def defaultIndexer(self) -> None:
        if self.indexerRunning:
            if self.indexerReversed:
                self.indexerMotor.setSpeed(-constants.kIndexerSpeed)
            else:
                self.indexerMotor.setSpeed(constants.kIndexerSpeed)
        else:
            self.indexerMotor.setSpeed(0)

        if self.stagingRunning:
            if self.stagingReversed:
                self.stagingMotor.setSpeed(-constants.kStagingSpeed)
            else:
                self.stagingMotor.setSpeed(constants.kStagingSpeed)
        else:
            self.stagingMotor.setSpeed(0)

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
