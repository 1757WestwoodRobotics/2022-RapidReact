from commands2 import SubsystemBase
from wpilib import PWMVictorSPX, PneumaticHub, PneumaticsModuleType, RobotBase, Solenoid
import constants


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeDeployed = False  # default to intake retracted and not reversed
        self.intakeReversed = False

        self.pneumaticsHub = PneumaticHub(1)

        self.intakeSolenoid = Solenoid(PneumaticsModuleType.REVPH, 1)
        self.intakeSolenoid.set(False)

        if RobotBase.isReal():
            # self.intakeMotor = WPI_TalonFX()
            pass
        else:
            self.intakeMotor = PWMVictorSPX(constants.kSimIntakeMotorPort)

    def toggleIntake(self) -> None:
        if self.intakeDeployed:
            self.retractIntake()
        else:
            self.deployIntake()

    def toggleReverseIntake(self) -> None:
        self.intakeReversed = not self.intakeReversed

    def reverseIntake(self) -> None:
        self.intakeReversed = True

    def unrevertIntake(self) -> None:
        self.intakeReversed = False

    def isIntakeDeployed(self) -> bool:
        return self.intakeDeployed

    def deployIntake(self) -> None:
        self.intakeDeployed = True
        self.intakeSolenoid.set(True)
        self.runIntake(self.intakeReversed)

    def retractIntake(self) -> None:
        self.intakeDeployed = False
        self.intakeSolenoid.set(False)
        self.stopIntake()

    def runIntake(self, reverse: bool) -> None:
        if reverse:
            # self.intakeMotor.set(-1.0)
            pass
        else:
            # self.intakeMotor.set(1.0)
            pass

    def stopIntake(self) -> None:
        # self.intakeMotor.set(0)
        pass

    def debugDeploy(self) -> None:
        self.intakeDeployed = True
        self.intakeSolenoid.set(True)
        self.stopIntake()
