#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
# Physical constants must have their units specified
#

import math
from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor
from units import units

# Physical parameters
kSwerveModuleCenterToCenterSideDistance = 21.5 * units.inches
kHalfSwerveModuleCenterToCenterSideDistance = (
    kSwerveModuleCenterToCenterSideDistance / 2
)
kSwerveModuleDistanceFromRobotCenter = pow(
    pow(kHalfSwerveModuleCenterToCenterSideDistance, 2)
    + pow(kHalfSwerveModuleCenterToCenterSideDistance, 2),
    0.5,
)  # c = (a^2 + b^2) ^ 0.5

# +x forward, +y right
kFrontLeftWheelPosition = Translation2d(
    kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
    -1 * kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
)
kFrontRightWheelPosition = Translation2d(
    kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
    kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
)
kBackLeftWheelPosition = Translation2d(
    -1 * kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
    -1 * kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
)
kBackRightWheelPosition = Translation2d(
    -1 * kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
    kHalfSwerveModuleCenterToCenterSideDistance.to(units.meters).magnitude,
)

kWheelDiameter = 4 * units.inches
kWheelRadius = kWheelDiameter / 2
kWheelCircumference = kWheelDiameter * math.pi
kWheelDistancePerRevolution = kWheelCircumference / units.revolution
kDriveGearingRatio = (48 / 16) * (16 / 28) * (60 / 15)
kSteerGearingRatio = (32 / 15) * (60 / 10)
kMaxMotorAngularSpeed = DCMotor.falcon500().freeSpeed * (units.radians / units.seconds)
kMaxWheelAngularSpeed = kMaxMotorAngularSpeed / kDriveGearingRatio
kMaxWheelSpeed = kWheelDistancePerRevolution * kMaxWheelAngularSpeed
kMaxSteerAngularSpeed = kMaxMotorAngularSpeed / kSteerGearingRatio
kMaxForwardSpeed = kMaxWheelSpeed
kMaxSidewaysSpeed = kMaxWheelSpeed
kMaxRotationAngularSpeed = (
    kMaxWheelSpeed / kSwerveModuleDistanceFromRobotCenter
) * units.radians  # omega = v / r

# Motors
kFrontLeftDriveMotorPort = 0
kFrontLeftSteerMotorPort = 1
kFrontRightDriveMotorPort = 2
kFrontRightSteerMotorPort = 3
kBackLeftDriveMotorPort = 4
kBackLeftSteerMotorPort = 5
kBackRightDriveMotorPort = 6
kBackRightSteerMotorPort = 7

# Encoders
kFrontLeftDriveEncoderPorts = (0, 1)
kFrontLeftSteerEncoderPorts = (2, 3)
kFrontRightDriveEncoderPorts = (4, 5)
kFrontRightSteerEncoderPorts = (6, 7)
kBackLeftDriveEncoderPorts = (8, 9)
kBackLeftSteerEncoderPorts = (10, 11)
kBackRightDriveEncoderPorts = (12, 13)
kBackRightSteerEncoderPorts = (14, 15)

kEncoderPulsesPerRevolution = 4096 * units.count / units.revolution
# Assumes the encoders are directly mounted on the wheel shafts
kWheelEncoderDistancePerPulse = (
    kWheelDistancePerRevolution / kEncoderPulsesPerRevolution
)
kSwerveEncoderAnglePerPulse = 1 / kEncoderPulsesPerRevolution

# Autonomous
kAutoDriveDistance = (
    (3 * units.revolutions)
    * kEncoderPulsesPerRevolution
    * kWheelEncoderDistancePerPulse
)  # three wheel revolutions
kAutoFrontwaysDistance = 24 * units.inches
kAutoSidewaysDistance = 24 * units.inches
kAutoDistanceThreshold = 6 * units.inches
kAutoDriveSpeedFactor = 0.5

# Operator Interface
kTranslationControllerPort = 0
kRotationControllerPort = 1
