from commands2 import SubsystemBase
from wpilib import PWMVictorSPX, RobotBase, Solenoid
import constants


class IntakeSubsystem(SubsystemBase):
    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.intakeDeployed = False  # default to intake retracted and off
        self.intakeRunning = False

        if RobotBase.isReal():
            pass
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
        self.intakeActive = True

    def retractIntake(self) -> None:
        self.intakeActive = False

    def runIntake(self) -> None:
        self.intakeMotor.set(1.0)

    def stopIntake(self) -> None:
        self.intakeMotor.set(0)
