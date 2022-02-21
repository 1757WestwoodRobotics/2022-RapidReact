from commands2 import SubsystemBase
from wpilib import PneumaticHub, PneumaticsModuleType, Solenoid, SmartDashboard
import constants
from util.helpfulIO import Falcon


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeDeployed = False  # default to intake retracted and not reversed
        self.intakeReversed = False

        self.pneumaticsHub = PneumaticHub(constants.kPneumaticsHubCanID)

        self.intakeSolenoid = Solenoid(
            PneumaticsModuleType.REVPH, constants.kIntakeSolenoidChannelId
        )
        self.intakeSolenoid.set(False)

        self.intakeMotor = Falcon(
            constants.kIntakeMotorName,
            constants.kIntakeMotorId,
            constants.kSimIntakeMotorPort,
        )

    def periodic(self) -> None:
        SmartDashboard.putBoolean(constants.kIntakeRunningKey, self.intakeDeployed)
        SmartDashboard.putBoolean(constants.kIntakeReversedKey, self.intakeReversed)

    def isIntakeReversed(self) -> bool:
        return self.intakeReversed

    def isIntakeDeployed(self) -> bool:
        return self.intakeDeployed

    def toggleIntake(self) -> None:
        if self.intakeDeployed:
            self.retractIntake()
        else:
            self.deployIntake()

    def reverseIntake(self) -> None:
        self.intakeReversed = True

    def unreverseIntake(self) -> None:
        self.intakeReversed = False

    def deployIntake(self) -> None:
        self.intakeDeployed = True
        self.intakeReversed = False
        self.intakeSolenoid.set(True)

    def retractIntake(self) -> None:
        self.intakeDeployed = False
        self.intakeSolenoid.set(False)
        print(
            f"Stopping. Variables: reverse is {self.intakeReversed} and running is {self.intakeDeployed}"
        )

    def debugDeploy(self) -> None:
        self.intakeDeployed = True
        self.intakeSolenoid.set(True)
        self.stopIntake()

    def defaultIntake(self) -> None:
        if self.intakeDeployed:
            self.intakeSolenoid.set(True)
            if self.intakeReversed:
                self.intakeMotor.setSpeed(constants.kIntakeSpeed)
            else:
                self.intakeMotor.setSpeed(-constants.kIntakeSpeed)
        else:
            self.intakeSolenoid.set(False)
            self.intakeMotor.setSpeed(0)
