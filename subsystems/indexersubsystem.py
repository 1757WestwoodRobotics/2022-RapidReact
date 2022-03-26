from enum import Enum, auto
from commands2 import SubsystemBase
from ctre import ControlMode, LimitSwitchNormal, LimitSwitchSource
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IndexerSubsystem(SubsystemBase):
    class Mode(Enum):
        Holding = auto()
        FeedForward = auto()
        Reversed = auto()
        Off = auto()

        def asString(self) -> str:
            mapping = {
                self.Holding: "Holding",
                self.FeedForward: "Feed Forward",
                self.Reversed: "Reversed",
                self.Off: "Off",
            }
            return mapping[self]

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexerMotor = createMotor(
            constants.kIndexerMotorId,
        )
        # INDEXER
        print(f"Initializing Falcon: {constants.kIndexerMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.indexerMotor.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.indexerMotor.config_kP(
                constants.kIndexerMotorPIDSlot,
                constants.kIndexerMotorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.indexerMotor.config_kI(
                constants.kIndexerMotorPIDSlot,
                constants.kIndexerMotorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.indexerMotor.config_kD(
                constants.kIndexerMotorPIDSlot,
                constants.kIndexerMotorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return

        if not ctreCheckError(
            "configReverseLimitSwitchSource",
            self.indexerMotor.configReverseLimitSwitchSource(
                LimitSwitchSource.Deactivated,
                LimitSwitchNormal.Disabled,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print(f"{constants.kIndexerMotorName} Falcon Initialization Complete")

        # STAGING
        self.stagingMotor = createMotor(
            constants.kStagingMotorId,
        )
        print(f"Initializing Falcon: {constants.kStagingMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.stagingMotor.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.stagingMotor.config_kP(
                constants.kStagingMotorPIDSlot,
                constants.kStagingMotorPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.stagingMotor.config_kI(
                constants.kStagingMotorPIDSlot,
                constants.kStagingMotorIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.stagingMotor.config_kD(
                constants.kStagingMotorPIDSlot,
                constants.kStagingMotorDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "configForwardLimitSwitchSource",
            self.indexerMotor.configForwardLimitSwitchSource(
                LimitSwitchSource.Deactivated,
                LimitSwitchNormal.Disabled,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print(f"{constants.kStagingMotorName} Falcon Initialization Complete")

        self.indexerSensor = self.stagingMotor.isRevLimitSwitchClosed
        self.stagingSensor = self.stagingMotor.isFwdLimitSwitchClosed
        self.state = self.Mode.Holding

    def periodic(self) -> None:
        if self.state == self.Mode.FeedForward:
            self.indexerMotor.set(
                ControlMode.Velocity,
                constants.kIndexerSpeed * constants.kTalonVelocityPerRPM,
            )
            self.stagingMotor.set(
                ControlMode.Velocity,
                constants.kStagingSpeed * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.Mode.Holding:
            self.indexerMotor.set(
                ControlMode.Velocity,
                constants.kIndexerSpeed * constants.kTalonVelocityPerRPM,
            )
            self.stagingMotor.set(
                ControlMode.Velocity,
                -constants.kStagingSpeed * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.Mode.Reversed:
            self.indexerMotor.set(
                ControlMode.Velocity,
                -constants.kIndexerSpeed * constants.kTalonVelocityPerRPM,
            )
            self.stagingMotor.set(
                ControlMode.Velocity,
                -constants.kStagingSpeed * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.Mode.Off:
            self.indexerMotor.set(ControlMode.Velocity, 0)
            self.stagingMotor.set(ControlMode.Velocity, 0)

    # Switches direction to reverse the ball path
    def motorsOff(self) -> None:
        self.state = self.Mode.Off

    def feedBallForward(self) -> None:
        self.state = self.Mode.FeedForward

    def holdBall(self) -> None:
        self.state = self.Mode.Holding

    def reversePath(self) -> None:
        self.state = self.Mode.Reversed
