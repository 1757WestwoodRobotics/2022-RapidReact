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
        self.runIntake()

    def isReversed(self) -> bool:
        return self.intakeReversed

    def retractIntake(self) -> None:
        self.intakeDeployed = False
        self.intakeSolenoid.set(False)
        self.stopIntake()
        print(
            f"Stopping. Variables: reverse is {self.intakeReversed} and running is {self.intakeDeployed}"
        )

    def runIntake(self) -> None:
        if self.isReversed():
            self.intakeMotor.setSpeed(-1 * constants.kIntakeSpeed)
            print(
                f"Reversing intake. Variables: reverse is {self.intakeReversed} and running is {self.intakeDeployed}",
            )
        else:
            self.intakeMotor.setSpeed(constants.kIntakeSpeed)
            print(
                f"Running intake. Variables: reverse is {self.intakeReversed} and running is {self.intakeDeployed}",
            )

    def stopIntake(self) -> None:
        self.intakeMotor.setSpeed(0)

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
