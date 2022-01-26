from commands2 import SubsystemBase
from ctre import WPI_TalonFX
from wpilib import PWMVictorSPX, PneumaticHub, PneumaticsModuleType, RobotBase, Solenoid
import constants


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeDeployed = False  # default to intake retracted and not reverseed
        self.intakeReversed = False

        self.pneumaticsHub = PneumaticHub(1)

        self.leftSolenoid = Solenoid(PneumaticsModuleType.REVPH, 0)
        self.leftSolenoid.set(False)

        self.rightSolenoid = Solenoid(PneumaticsModuleType.REVPH, 1)
        self.rightSolenoid.set(False)

        if RobotBase.isReal():
            self.intakeMotor = WPI_TalonFX(constants.kIntakeMotorId)
        else:
            self.intakeMotor = PWMVictorSPX(constants.kSimIntakeMotorPort)

    def toggleIntakeDeploy(self) -> None:
        self.intakeDeploy = not self.intakeDeploy

    def toggleIntakeMotor(self) -> None:
        self.intakeRunning = not self.intakeRunning

    def isIntakeDeployed(self) -> bool:
        return self.intakeDeployed

    def isIntakeRunning(self) -> bool:
        return self.intakeRunning

    def deployIntake(self) -> None:
        self.intakeDeployed = True
        self.leftSolenoid.set(True)
        self.rightSolenoid.set(True)
        self.runIntake(self.intakeReversed)

    def retractIntake(self) -> None:
        self.intakeDeployed = False
        self.leftSolenoid.set(False)
        self.rightSolenoid.set(False)
        self.stopIntake()

    def runIntake(self, reverse: bool) -> None:
        if reverse:
            self.intakeMotor.set(-1.0)
        else:
            self.intakeMotor.set(1.0)

    def stopIntake(self) -> None:
        self.intakeMotor.set(0)
