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
            path.join(path.dirname(path.realpath(__file__)), "ControlScheme.json"),
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

        self.coordinateModeControl = getButtonBindingOfName("coordinateModeControl")
        self.resetSwerveControl = getButtonBindingOfName("resetSwerveControl")
        self.targetRelativeCoordinateModeControl = getButtonBindingOfName("targetRelativeCoordinateModeControl")

        self.driveToTargetControl = getButtonBindingOfName("driveToTargetControl")

        # self.chassisControls = HolonomicInput(
        #     Invert(
        #         Deadband(
        #             lambda: self.translationController.getY(GenericHID.Hand.kLeftHand),
        #             constants.kKeyboardJoystickDeadband,
        #         )
        #     ),
        #     Invert(
        #         Deadband(
        #             lambda: self.translationController.getX(GenericHID.Hand.kLeftHand),
        #             constants.kKeyboardJoystickDeadband,
        #         )
        #     ),
        #     Invert(
        #         Deadband(
        #             lambda: self.rotationController.getX(GenericHID.Hand.kRightHand),
        #             constants.kKeyboardJoystickDeadband,
        #         )
        #     ),
        # )

        self.chassisControls = HolonomicInput(
            Invert(
                Deadband(
                    getAxisBindingOfName("forwardsBackwards"),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName("sideToSide"),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    getAxisBindingOfName("rotation"),
                    constants.kXboxJoystickDeadband,
                )
            ),
        )
