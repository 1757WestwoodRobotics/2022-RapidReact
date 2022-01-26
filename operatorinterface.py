import typing
import json

from os import path
from wpilib import Joystick

import constants

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


class HolonomicInput:
    def __init__(
        self,
        forwardsBackwards: AnalogInput,
        sideToSide: AnalogInput,
        rotation: AnalogInput,
    ) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide
        self.rotation = rotation


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
            i[0] for i in controlScheme.values()
        )  # set ensures no duplicates

        controllers = {}

        for num in controllerNumbers:
            controllers[num] = Joystick(num)

        def getButtonBindingOfName(name: str) -> typing.Tuple[Joystick, int]:
            binding = controlScheme[name]
            return (controllers[binding[0]], binding[1]["Button"])

        def getAxisBindingOfName(name: str) -> AnalogInput:
            binding = controlScheme[name]
            return lambda: controllers[binding[0]].getRawAxis(binding[1]["Axis"])

        self.fieldRelativeCoordinateModeControl = getButtonBindingOfName(
            constants.kFieldRelativeCoordinateModeControlButtonName
        )
        self.resetSwerveControl = getButtonBindingOfName(
            constants.kResetSwerveControlButtonName
        )
        self.targetRelativeCoordinateModeControl = getButtonBindingOfName(
            constants.kTargetRelativeCoordinateModeControlButtonName
        )
        self.driveToTargetControl = getButtonBindingOfName(
            constants.kDriveToTargetControlButtonName
        )

        self.runIntakeMotorControl = getButtonBindingOfName(
            constants.kRunIntakeButtonName
        )

        self.launchCargo = getButtonBindingOfName(constants.kLaunchCargoButtonName)
        self.toggleIntake = getButtonBindingOfName(constants.kToggleIntakeButtonName)
        self.toggleClimberBreak = getButtonBindingOfName(
            constants.kToggleClimberBreakButtonName
        )

        self.chassisControls = HolonomicInput(
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisForwardsBackwardsAxisName),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisSideToSideAxisName),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName(constants.kChassisRotationAxisName),
                    constants.kXboxJoystickDeadband,
                )
            ),
        )
