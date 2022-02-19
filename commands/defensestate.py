from commands2 import CommandBase
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.geometry import Rotation2d


class DefenseState(CommandBase):
    def __init__(self, drive: DriveSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive

        self.addRequirements([self.drive])

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        for module in self.drive.modules:
            module.setWheelLinearVelocityTarget(0)
        self.drive.frontLeftModule.setSwerveAngleTarget(
            self.drive.frontLeftModule.optimizedAngle(Rotation2d.fromDegrees(-45))
        )
        self.drive.frontRightModule.setSwerveAngleTarget(
            self.drive.frontRightModule.optimizedAngle(Rotation2d.fromDegrees(45))
        )
        self.drive.backLeftModule.setSwerveAngleTarget(
            self.drive.frontLeftModule.optimizedAngle(Rotation2d.fromDegrees(-135))
        )
        self.drive.backRightModule.setSwerveAngleTarget(
            self.drive.frontLeftModule.optimizedAngle(Rotation2d.fromDegrees(135))
        )
