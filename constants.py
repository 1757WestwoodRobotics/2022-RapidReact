#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
# Physical constants must have their units specified
#

import math
from wpimath.system.plant import DCMotor
from pyfrc.physics.units import units

# Physical parameters
kDriveTrainMotorsPerSide = 2
kTrackWidth = 30 * units.inches
kWheelDiameter = 6 * units.inches
kWheelRadius = kWheelDiameter / 2
kWheelCircumference = kWheelDiameter * math.pi
kWheelDistancePerRevolution = kWheelCircumference / units.revolution
kGearingRatio = 8
kMaxMotorAngularSpeed = DCMotor.falcon500().freeSpeed * \
    (units.radians / units.seconds)
kMaxWheelAngularSpeed = kMaxMotorAngularSpeed / kGearingRatio
kMaxWheelSpeed = kWheelDistancePerRevolution * kMaxWheelAngularSpeed
kMaxForwardSpeed = kMaxWheelSpeed
kMaxSidewaysSpeed = 0 * units.meters / units.second  # differential drive
kMaxRotationAngularSpeed = (kMaxWheelSpeed / (kTrackWidth / 2)) * units.radians

# Motors
kFrontLeftMotorPort = 0
kBackLeftMotorPort = 1
kFrontRightMotorPort = 2
kBackRightMotorPort = 3
kInvertLeftMotors = False
kInvertRightMotors = True

# Encoders
kLeftEncoderPorts = (0, 1)
kRightEncoderPorts = (2, 3)
kLeftEncoderReversed = False
kRightEncoderReversed = True

kEncoderPulsesPerRevolution = 1024 * units.count / units.revolution
# Assumes the encoders are directly mounted on the wheel shafts
kEncoderDistancePerPulse = kWheelDistancePerRevolution / kEncoderPulsesPerRevolution

# Autonomous
kAutoDriveDistance = (3 * units.revolutions) * kEncoderPulsesPerRevolution * \
    kEncoderDistancePerPulse  # three wheel revolutions
kAutoBackupDistance = 20 * units.inches
kAutoDistanceThreshold = 6 * units.inches
kAutoDriveSpeedFactor = 0.5

# Operator Interface
kDriverControllerPort = 0
