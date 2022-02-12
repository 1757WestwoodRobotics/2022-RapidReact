from commands2 import SubsystemBase, TrapezoidProfileSubsystem
from wpilib import PWMVictorSPX, RobotBase
from ctre import WPI_TalonFX

import constants


class IndexerSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexerRunning = False
        self.indexerReversed = False

        if RobotBase.isReal():
            self.indexerMotor = WPI_TalonFX(constants.kIndexerMotorId)
        else:
            self.indexerMotor = PWMVictorSPX(constants.kSimIndexerMotorPort)

    def isIndexerRunning(self) -> bool:
        return self.indexerRunning

    def isIndexerReversed(self) -> bool:
        return self.indexerReversed

    def runIndexer(self) -> None:
        if not self.isIndexerRunning() or self.isIndexerReversed():
            self.indexerRunning = True
            self.indexerReversed = False
            self.indexerMotor.set(1.0)
        else:
            self.stopIndexer()

    def reverseIndexer(self) -> None:
        if not self.isIndexerRunning() or not self.isIndexerReversed():
            self.indexerRunning = True
            self.indexerReversed = True
            self.indexerMotor.set(-1.0)
        else:
            self.stopIndexer()

    def stopIndexer(self) -> None:
        self.indexerRunning = False
        self.indexerMotor.set(0)
