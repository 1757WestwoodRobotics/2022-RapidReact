from wpilib import Joystick, XboxController
from wpilib.interfaces import GenericHID

import typing

import constants


AnalogInput = typing.Callable[[], float]


def Deadband(input: AnalogInput, deadband: float) -> AnalogInput:
    def withDeadband() -> float:
        value = input()
        if abs(value) <= deadband:
            return 0
        else:
            return value

    return withDeadband


def Invert(input: AnalogInput) -> AnalogInput:
    def invert() -> float:
        return -1 * input()

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
        self.xboxController = XboxController(constants.kXboxControllerPort)
        self.translationController = Joystick(constants.kTranslationControllerPort)
        self.rotationController = Joystick(constants.kRotationControllerPort)

        self.coordinateModeControl = (
            self.xboxController,
            XboxController.Button.kA.value,
        )

        self.resetSwerveControl = (
            self.xboxController,
            XboxController.Button.kX.value,
        )

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
                    lambda: self.xboxController.getY(GenericHID.Hand.kLeftHand),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    lambda: self.xboxController.getX(GenericHID.Hand.kLeftHand),
                    constants.kXboxJoystickDeadband,
                )
            ),
            Invert(
                Deadband(
                    lambda: self.xboxController.getX(GenericHID.Hand.kRightHand),
                    constants.kXboxJoystickDeadband,
                )
            ),
        )
