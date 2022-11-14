from commands2 import SubsystemBase
from ctre.led import CANdle, RainbowAnimation, StrobeAnimation
from wpilib import SmartDashboard, RobotState
import constants


class LightSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.candle = CANdle(constants.kCANdleID, constants.kCANivoreName)

        self.disabledAnimation = RainbowAnimation(1, 0.5, 68)
        self.noBallAnimation = StrobeAnimation(255, 0, 0)  # red
        self.twoBallOnTargetAnimation = StrobeAnimation(0, 255, 0)  # green
        self.twoBallOffTargetAnimation = StrobeAnimation(255, 0, 255)  # pink
        self.oneBallOnTargetAnimation = StrobeAnimation(0, 0, 255)  # blue
        self.oneBallOffTargetAnimation = StrobeAnimation(255, 255, 0)  # yellow

    def periodic(self) -> None:
        if RobotState.isDisabled():
            self.candle.animate(self.disabledAnimation)
        else:
            if SmartDashboard.getBoolean(constants.kReadyToFireKey, False):  # one ball
                if SmartDashboard.getBoolean(
                    constants.kDualBallKey, False
                ):  # two balls
                    if SmartDashboard.getBoolean(
                        constants.kShootingTurretOnTargetKey, False
                    ):  # two balls, on target
                        self.candle.animate(self.twoBallOnTargetAnimation)
                    else:
                        self.candle.animate(self.twoBallOffTargetAnimation)
                else:  # only one ball
                    if SmartDashboard.getBoolean(
                        constants.kShootingTurretOnTargetKey, False
                    ):
                        self.candle.animate(self.oneBallOnTargetAnimation)
                    else:
                        self.candle.animate(self.oneBallOffTargetAnimation)
            else:  # no ball
                self.candle.animate(self.noBallAnimation)
