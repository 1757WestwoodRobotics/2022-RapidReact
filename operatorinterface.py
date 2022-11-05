import typing
import json

from os import path
from wpilib import Joystick, Preferences

import constants
from util.convenientmath import map_range, number

AnalogInput = typing.Callable[[], float]


def Deadband(inputFn: AnalogInput, deadband: float) -> AnalogInput:
    def withDeadband() -> float:
        value = inputFn()
        if abs(value) <= deadband:
            return 0
        else:
            return value

    return withDeadband


def Invert(inputFn: AnalogInput) -> AnalogInput:
    def invert() -> float:
        return -1 * inputFn()

    return invert


def MapRange(
    inputFn: AnalogInput,
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number,
) -> AnalogInput:
    return lambda: map_range(inputFn(), inputMin, inputMax, outputMin, outputMax)


def Multiply(a: AnalogInput, b: AnalogInput) -> AnalogInput:
    return lambda: a() * b()


class HolonomicInput:
    def __init__(
        self,
        forwardsBackwards: AnalogInput,
        sideToSide: AnalogInput,
        rotationX: AnalogInput,
        rotationY: AnalogInput,
    ) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide
        self.rotationX = rotationX
        self.rotationY = rotationY


class Control2D:
    def __init__(self, forwardsBackwards: AnalogInput, sideToSide: AnalogInput) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide


class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    def __init__(self) -> None:
        with open(
            path.join(
                path.dirname(path.realpath(__file__)),
                constants.kControllerMappingFilename,
            ),
            "r",
            encoding="utf-8",
        ) as file:
            controlScheme = json.load(file)

        controllerNumbers = set(
            [0,1,2,3,4,5]
        )  # set ensures no duplicates
        print(f"Looking for controllers: {controllerNumbers} ...")

        controllers = {}

        self.prefs = Preferences

        for control in controlScheme:
            binding = controlScheme[control]
            self.prefs.setInt(control + " controller", binding[0])
            if "Button" in binding[1].keys():
                self.prefs.setInt(control + " button", binding[1]["Button"])
            elif "Axis" in binding[1].keys():
                self.prefs.setInt(control + " axis", binding[1]["Axis"])

        for num in controllerNumbers:
            controller = Joystick(num)
            print(
                f"Found Controller {num}:{controller.getName()}\n\tAxis: {controller.getAxisCount()}\n\tButtons: {controller.getButtonCount()}\n\tPoV Hats: {controller.getPOVCount()}"
            )
            controllers[num] = controller

        def getButtonBindingOfName(
            name: str,
        ) -> typing.Callable[[], typing.Tuple[Joystick, int]]:
            return lambda: (
                controllers[self.prefs.getInt(name + " controller")],
                self.prefs.getInt(name + " button"),
            )

        def getAxisBindingOfName(name: str) -> AnalogInput:
            return lambda: controllers[
                self.prefs.getInt(name + " controller")
            ].getRawAxis(self.prefs.getInt(name + " axis"))

        self.fieldRelativeCoordinateModeControl = getButtonBindingOfName(
            constants.kFieldRelativeCoordinateModeControlButtonName
        )
        self.resetGyro = getButtonBindingOfName(constants.kResetGyroButtonName)
        self.targetRelativeCoordinateModeControl = getButtonBindingOfName(
            constants.kTargetRelativeCoordinateModeControlButtonName
        )
        self.driveToTargetControl = getButtonBindingOfName(
            constants.kDriveToTargetControlButtonName
        )

        self.moveBothClimbersToMiddleRungCapturePosition = getButtonBindingOfName(
            constants.kMoveBothClimbersToMiddleRungCapturePositionName
        )

        self.pivotBothClimbers = getButtonBindingOfName(
            constants.kPivotBothClimbersButtonName
        )

        self.moveBothClimbersToMiddleRungHangPosition = getButtonBindingOfName(
            constants.kMoveBothClimbersToMiddleRungHangPositionName
        )

        self.holdBothClimbersPosition = getButtonBindingOfName(
            constants.kHoldBothClimbersPositionName
        )

        self.rightClimberToNextRungCapturePosition = getButtonBindingOfName(
            constants.kRightClimberToNextRungCapturePositionButtonName
        )

        self.leftClimberToNextRungCapturePosition = getButtonBindingOfName(
            constants.kLeftClimberToNextRungCapturePositionButtonName
        )

        self.rightClimberToHangingPosition = getButtonBindingOfName(
            constants.kRightClimberToHangingPositionButtonName
        )

        self.leftClimberToHangingPosition = getButtonBindingOfName(
            constants.kLeftClimberToHangingPositionButtonName
        )
        self.deployIntakeControl = getButtonBindingOfName(
            constants.kDeployIntakeButtonName
        )

        self.reverseBallPath = getButtonBindingOfName(constants.kReverseBallPathName)

        self.autoBallIntakeControl = getButtonBindingOfName(
            constants.kAutoBallIntakeName
        )

        self.shootBall = getButtonBindingOfName(constants.kShootBallButtonName)

        self.increaseSpeed = getButtonBindingOfName(
            constants.kIncreaseShootingSpeedButonName
        )
        self.decreaseSpeed = getButtonBindingOfName(
            constants.kDecreaseShootingSpeedButtonName
        )
        self.resetSpeed = getButtonBindingOfName(
            constants.kResetShootingSpeedButtonName
        )

        self.defenseStateControl = getButtonBindingOfName("defenseStateControl")

        self.turboSpeed = getButtonBindingOfName(constants.kTurboSpeedButtonName)

        self.stopMovingParts = getButtonBindingOfName(
            constants.kStopMovingPartsButtonName
        )

        self.chassisControls = HolonomicInput(
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisForwardsBackwardsAxisName),
                    constants.kXboxJoystickDeadband,
                ),
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisSideToSideAxisName),
                    constants.kXboxJoystickDeadband,
                ),
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisRotationXAxisName),
                    constants.kXboxJoystickDeadband,
                ),
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisRotationYAxisName),
                    constants.kXboxJoystickDeadband,
                )
            ),
        )

        self.shooterOffset = Control2D(
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kTurretAngleOffsetAxisName),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kShootingDistanceOffsetAxisName),
                    constants.kXboxJoystickDeadband,
                )
            ),
        )
