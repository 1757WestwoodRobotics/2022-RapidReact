from commands2 import SubsystemBase
from wpilib import PneumaticHub, PneumaticsModuleType, Solenoid
import constants
from util.helpfulIO import Falcon


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeDeployed = False  # default to intake retracted and not reversed
        self.intakeReversed = False

        self.pneumaticsHub = PneumaticHub(1)

        self.intakeSolenoid = Solenoid(
            PneumaticsModuleType.REVPH, constants.kPneumaticsHubCanID
        )
        self.intakeSolenoid.set(False)

        self.intakeMotor = Falcon(
            constants.kIntakeMotorName,
            constants.kIndexerMotorId,
            constants.kSimIntakeMotorPort,
        )

    def toggleIntake(self) -> None:
        if self.intakeDeployed:
            self.retractIntake()
        else:
            self.deployIntake()

    def toggleReverseIntake(self) -> None:
        self.intakeReversed = not self.intakeReversed

    def reverseIntake(self) -> None:
        self.intakeReversed = True

    def unreverseIntake(self) -> None:
        self.intakeReversed = False

    def isIntakeDeployed(self) -> bool:
        return self.intakeDeployed

    def deployIntake(self) -> None:
        self.intakeDeployed = True
        self.intakeReversed = False
        self.intakeSolenoid.set(True)
        self.runIntake(self.intakeReversed)

    def retractIntake(self) -> None:
        self.intakeDeployed = False
        self.intakeSolenoid.set(False)
        self.stopIntake()

    def runIntake(self, reverse: bool) -> None:
        if reverse:
            self.intakeMotor.setSpeed(-1000)
        else:
            self.intakeMotor.setSpeed(1000)

    def stopIntake(self) -> None:
        self.intakeMotor.setSpeed(0)

    def debugDeploy(self) -> None:
        self.intakeDeployed = True
        self.intakeSolenoid.set(True)
        self.stopIntake()
