from commands2 import SubsystemBase
from wpilib import PowerDistribution, SmartDashboard

import constants


class SystemLogSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.pdh = PowerDistribution()

    def periodic(self) -> None:
        SmartDashboard.putNumberArray(
            constants.kRobotPowerChannelsKey,
            [self.pdh.getCurrent(i) for i in range(24)],
        )

        SmartDashboard.putNumber(
            constants.kRobotVoltageChannelKey,
            self.pdh.getVoltage()
        )
