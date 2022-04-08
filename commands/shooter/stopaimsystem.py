from commands2 import CommandBase, ParallelCommandGroup
from wpimath.geometry import Rotation2d

from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.indexersubsystem import IndexerSubsystem


class StopIndexer(CommandBase):
    def __init__(self, indexer: IndexerSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexer = indexer
        self.addRequirements([self.indexer])

    def execute(self) -> None:
        self.indexer.motorsOff()

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True


class StopAimSystem(CommandBase):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.addRequirements([self.shooter])

    def execute(self) -> None:
        self.shooter.shootingMotor.neutralOutput()
        self.shooter.rotateTurret(Rotation2d.fromDegrees(0))


class StopMovingParts(ParallelCommandGroup):
    def __init__(self, indexer: IndexerSubsystem, shooter: ShooterSubsystem):
        self.setName(__class__.__name__)
        super().__init__(StopIndexer(indexer), StopAimSystem(shooter))
