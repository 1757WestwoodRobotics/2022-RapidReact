from commands2 import SubsystemBase
from ctre.led import CANdle
from wpilib import SmartDashboard
import constants


class LightSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.candle = CANdle(constants.kCANdleID, constants.kCANivoreName)

    def periodic(self) -> None:
        if SmartDashboard.getBoolean(constants.kReadyToFireKey, False):
            if SmartDashboard.getBoolean(constants.kDualBallKey, False):
                if SmartDashboard.getBoolean(
                    constants.kShootingTurretOnTargetKey, False
                ):
                    self.candle.setLEDs(0, 255, 0)  # green
                else:
                    self.candle.setLEDs(255, 255, 255)  # white
            else:  # only one ball
                if SmartDashboard.getBoolean(
                    constants.kShootingTurretOnTargetKey, False
                ):
                    self.candle.setLEDs(0, 0, 255)  # blue
                else:
                    self.candle.setLEDs(255, 255, 0)  # yellow
        else:  # no ball
            self.candle.setLEDs(255, 0, 0)  # red
