"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

import math
from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor

# Basic units
kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""

kRadiansPerRevolution = 2 * math.pi
"""radians / revolution"""

kDegeersPerRevolution = 360
"""degrees / revolution"""

kRadiansPerDegree = kRadiansPerRevolution / kDegeersPerRevolution
"""radians / degree"""

kMillisecondsPerSecond = 1000 / 1
"""milliseconds / second"""

# Debug parameters
kPrintFrequency = 2
""" 1 / second"""

kPrintPeriod = 1 / kPrintFrequency

# Physical parameters
kSwerveModuleCenterToCenterSideDistance = 21.5 * kMetersPerInch
"""meters"""

kHalfSwerveModuleCenterToCenterSideDistance = (
    kSwerveModuleCenterToCenterSideDistance / 2
)
"""meters"""

kSwerveModuleDistanceFromRobotCenter = pow(
    pow(kHalfSwerveModuleCenterToCenterSideDistance, 2)
    + pow(kHalfSwerveModuleCenterToCenterSideDistance, 2),
    0.5,
)
"""meters (c = (a^2 + b^2) ^ 0.5)"""


# +x forward, +y right
kFrontLeftWheelPosition = Translation2d(
    kHalfSwerveModuleCenterToCenterSideDistance,
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
)
kFrontRightWheelPosition = Translation2d(
    kHalfSwerveModuleCenterToCenterSideDistance,
    kHalfSwerveModuleCenterToCenterSideDistance,
)
kBackLeftWheelPosition = Translation2d(
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
)
kBackRightWheelPosition = Translation2d(
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
    kHalfSwerveModuleCenterToCenterSideDistance,
)

kWheelDiameter = 4 * kMetersPerInch
"""meters"""

kWheelRadius = kWheelDiameter / 2
"""meters"""

kWheelCircumference = kWheelRadius * 2 * math.pi
"""meters"""

kWheelDistancePerRevolution = kWheelCircumference
"""meters / revolution"""

kWheelDistancePerRadian = kWheelDistancePerRevolution / kRadiansPerRevolution
"""meters / radian"""

kDriveGearingRatio = (48 / 16) * (16 / 28) * (60 / 15)
"""dimensionless"""

kSteerGearingRatio = (32 / 15) * (60 / 10)
"""dimensionless"""

kMaxMotorAngularVelocity = DCMotor.falcon500().freeSpeed
"""radians / second"""

kMaxWheelAngularVelocity = kMaxMotorAngularVelocity / kDriveGearingRatio
"""radians / second"""

kMaxWheelLinearVelocity = kWheelDistancePerRadian * kMaxWheelAngularVelocity
"""meters / second"""

kMaxSteerAngularVelocity = kMaxMotorAngularVelocity / kSteerGearingRatio
"""radians / second"""

kMaxForwardLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxSidewaysLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxRotationAngularVelocity = (
    kMaxWheelLinearVelocity / kSwerveModuleDistanceFromRobotCenter
)
"""radians / second (omega = v / r)"""

kFrontLeftModuleName = "front_left"
kFrontRightModuleName = "front_right"
kBackLeftModuleName = "back_left"
kBackRightModuleName = "back_right"

# Motors
kFrontLeftDriveMotorSimPort = 0
kFrontLeftSteerMotorSimPort = 1
kFrontRightDriveMotorSimPort = 2
kFrontRightSteerMotorSimPort = 3
kBackLeftDriveMotorSimPort = 4
kBackLeftSteerMotorSimPort = 5
kBackRightDriveMotorSimPort = 6
kBackRightSteerMotorSimPort = 7

kFrontLeftDriveMotorId = 10
kFrontLeftSteerMotorId = 11
kFrontRightDriveMotorId = 12
kFrontRightSteerMotorId = 13
kBackLeftDriveMotorId = 14
kBackLeftSteerMotorId = 15
kBackRightDriveMotorId = 16
kBackRightSteerMotorId = 17

# Encoders
kFrontLeftDriveEncoderSimPorts = (0, 1)
kFrontLeftSteerEncoderSimPorts = (2, 3)
kFrontRightDriveEncoderSimPorts = (4, 5)
kFrontRightSteerEncoderSimPorts = (6, 7)
kBackLeftDriveEncoderSimPorts = (8, 9)
kBackLeftSteerEncoderSimPorts = (10, 11)
kBackRightDriveEncoderSimPorts = (12, 13)
kBackRightSteerEncoderSimPorts = (14, 15)

kFrontLeftSteerEncoderId = 40
kFrontRightSteerEncoderId = 41
kBackLeftSteerEncoderId = 42
kBackRightSteerEncoderId = 43

kCANcoderPulsesPerRevolution = 4096
"""pulses / revolution"""

kCANcoderPulsesPerRadian = kCANcoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kTalonEncoderPulsesPerRevolution = 2048
"""pulses / revolution"""

kTalonEncoderPulsesPerRadian = kTalonEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kDriveEncoderPulsesPerRadian = kDriveEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerMeter = kDriveEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kWheelEncoderPulsesPerRevolution = kDriveEncoderPulsesPerRevolution * kDriveGearingRatio
"""pulses / revolution"""

kWheelEncoderPulsesPerRadian = kWheelEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kWheelEncoderPulsesPerMeter = kWheelEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kSteerEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kSteerEncoderPulsesPerRadian = kSteerEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kSwerveEncoderPulsesPerRevolution = (
    kSteerEncoderPulsesPerRevolution * kSteerGearingRatio
)
"""pulses / revolution"""

kSwerveEncoderPulsesPerRadian = (
    kSwerveEncoderPulsesPerRevolution / kRadiansPerRevolution
)
"""pulses / radian"""

# CTRE
k100MillisecondsPerSecond = 10 / 1  # there are 10 groups of 100 milliseconds per second
"""100 milliseconds / second
   CTRE reports velocities in units of (quantity / 100 milliseconds)
   This factor is used to convert to (quantity / 1 second)
"""
kConfigurationTimeoutLimit = int(5 * kMillisecondsPerSecond)
"""milliseconds"""

kDrivePIDSlot = 0
kDrivePGain = 0.01
kDriveIGain = 0.0
kDriveDGain = 0.0

kSteerPIDSlot = 0
kSteerPGain = 0.6
kSteerIGain = 0.0
kSteerDGain = 12.0

"""
To determine encoder offsets (with robot ON and DISABLED):
  1. Rotate all swerve modules so that the wheels:
     * are running in the forwards-backwards direction
     * have the wheel bevel gears facing inwards towards the
       center-line of the robot
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""
kFrontLeftAbsoluteEncoderOffset = 314.912
"""degrees"""

kFrontRightAbsoluteEncoderOffset = 5.713
"""degrees"""

kBackLeftAbsoluteEncoderOffset = 19.424
"""degrees"""

kBackRightAbsoluteEncoderOffset = 142.998
"""degrees"""

# Autonomous
kAutoDriveDistance = 3 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5

# Operator Interface
kXboxControllerPort = 0
kTranslationControllerPort = 1
kRotationControllerPort = 2

kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""
