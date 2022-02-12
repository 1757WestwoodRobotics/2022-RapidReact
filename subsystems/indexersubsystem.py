from commands2 import SubsystemBase

import constants
from wpilib import WPI_TalonFX, PWMVictorSPX, RobotBase


class IndexerSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        if RobotBase.isReal():
            self.indexMotor = WPI_TalonFX()
            pass
        else:
            self.indexMotor = PWMVictorSPX(constants.kSimIndexerMotorPort)

    def runIndexer(self) -> None:
        self.indexMotor.set(1.0)

    def stopIndexer(self) -> None:
        self.indexMotor.set(0)

    def reverseIndexer(self) -> None:
        self.indexMotor.set(-1.0)
