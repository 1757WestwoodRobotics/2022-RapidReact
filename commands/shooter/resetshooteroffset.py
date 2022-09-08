from commands2 import CommandBase
from wpilib import SmartDashboard

import constants


class ResetShooterOffset(CommandBase):
    def __init__(self) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        SmartDashboard.putNumber(constants.kWheelSpeedTweakKey, 0)

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
