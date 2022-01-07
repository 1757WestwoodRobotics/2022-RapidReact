import typing

from wpilib import Joystick, XboxController
from wpilib.interfaces import GenericHID

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
        self.xboxController = XboxController(constants.kXboxControllerPort)
        self.translationController = Joystick(constants.kTranslationControllerPort)
        self.rotationController = Joystick(constants.kRotationControllerPort)

        self.fieldRelativeCoordinateModeControl = (
            self.xboxController,
            XboxController.Button.kBumperRight.value,
        )

        self.targetRelativeCoordinateModeControl = (
            self.xboxController,
            XboxController.Button.kBumperLeft.value,
        )

        self.resetSwerveControl = (
            self.xboxController,
            XboxController.Button.kX.value,
        )

        self.driveToTargetControl = (
            self.xboxController,
            XboxController.Button.kY.value,
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
