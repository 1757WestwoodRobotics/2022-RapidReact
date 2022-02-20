from commands2 import ParallelCommandGroup
from commands.indexer.holdball import HoldBall
from commands.intake.normalintake import NormalIntake

from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.intakesubsystem import IntakeSubsystem


class NormalBallPath(ParallelCommandGroup):
    def __init__(self, intake: IntakeSubsystem, indexer: IndexerSubsystem):
        super().__init__(HoldBall(indexer), NormalIntake(intake))
        self.setName(__class__.__name__)
