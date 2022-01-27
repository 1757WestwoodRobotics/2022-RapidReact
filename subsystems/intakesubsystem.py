from commands2 import SubsystemBase
from wpilib import PWMVictorSPX, PneumaticHub, PneumaticsModuleType, RobotBase, Solenoid
import constants


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeDeployed = False  # default to intake retracted and off

        self.pneumaticsHub = PneumaticHub(1)

        self.leftSolenoid = Solenoid(PneumaticsModuleType.REVPH, 0)
        self.leftSolenoid.set(False)

        self.rightSolenoid = Solenoid(PneumaticsModuleType.REVPH, 1)
        self.rightSolenoid.set(False)

        if RobotBase.isReal():
            pass
        else:
            self.intakeMotor = PWMVictorSPX(constants.kSimIntakeMotorPort)

    def toggleIntake(self) -> None:
        self.intakeDeployed = not self.intakeDeployed

    def isIntakeDeployed(self) -> bool:
        return self.intakeDeployed

    def deployIntake(self) -> None:
        self.intakeDeployed = True
        self.leftSolenoid.set(True)
        self.rightSolenoid.set(True)
        self.runIntake()

    def retractIntake(self) -> None:
        self.intakeDeployed = False
        self.leftSolenoid.set(False)
        self.rightSolenoid.set(False)
        self.stopIntake()

    def runIntake(self) -> None:
        self.intakeRunning = True
        self.intakeMotor.set(1.0)

    def stopIntake(self) -> None:
        self.intakeRunning = False
        self.intakeMotor.set(0)
