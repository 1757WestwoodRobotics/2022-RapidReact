from commands2 import CommandBase
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem


class AutoBallIntake(CommandBase):
    def __init__(self, drive: DriveSubsystem, intake: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.intake = intake
        # finish vision subsystem first
