from commands2 import CommandBase
from wpilib import SmartDashboard

import constants


class DecreaseShooterSpeed(CommandBase):
    def __init__(self) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        num = SmartDashboard.getNumber(constants.kWheelSpeedTweakKey, 0)
        SmartDashboard.putNumber(
            constants.kWheelSpeedTweakKey, num - constants.kWheelSpeedTweakAmount
        )

    # pylint: disable-next=no-self-use
    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    # pylint: disable-next=no-self-use
    def isFinished(self) -> bool:
        return True