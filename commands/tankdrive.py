import typing
from commands2 import CommandBase
from subsystems.drivesubsystem import DriveSubsystem
import constants
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units


class TankDrive(CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        left: typing.Callable[[], float],
        right: typing.Callable[[], float],
    ) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_FALCON_500,
            robot_mass=124 * units.lbs,
            gearing=constants.kDriveGearingRatio,
            nmotors=2,
            x_wheelbase=constants.kSwerveModuleCenterToCenterSideDistance * units.meters,
            wheel_diameter=constants.kWheelDiameter * units.meters,
        )

        self.drive = drive
        self.left = left
        self.right = right

        self.addRequirements([self.drive])

    def execute(self) -> None:
        l = self.left()
        r = -self.right()

        target_pos = self.drivetrain.calculate(l, r, 0.02)

        self.drive.arcadeDriveWithFactors(
            target_pos.X() * 12,
            target_pos.Y() * 12,
            target_pos.rotation().radians(),
            DriveSubsystem.CoordinateMode.RobotRelative,
        )
