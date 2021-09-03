#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean constants. Don't use this for any other purpose!
#

import math
from pyfrc.physics.units import units

# Physical parameters
kDriveTrainMotorsPerSide = 2
kTrackWidth = (30 * units.inches).to(units.meter).magnitude
kWheelDiameter = (6 * units.inches).to(units.meter).magnitude
kWheelRadius = kWheelDiameter / 2
kWheelCircumference = (kWheelDiameter * math.pi)
kGearingRatio = 8

# Motors
kLeftMotor1Port = 0
kLeftMotor2Port = 1
kRightMotor1Port = 2
kRightMotor2Port = 3

# Encoders
kLeftEncoderPorts = (0, 1)
kRightEncoderPorts = (2, 3)
kLeftEncoderReversed = False
kRightEncoderReversed = True

kEncoderPulsesPerRevolution = 1024
# Assumes the encoders are directly mounted on the wheel shafts
kEncoderDistancePerPulse = kWheelCircumference / kEncoderPulsesPerRevolution

# Autonomous
kAutoDriveDistance = 3 * kEncoderPulsesPerRevolution * \
    kEncoderDistancePerPulse  # three wheel revolutions
kAutoBackupDistance = (20 * units.inches).to(units.meter).magnitude
kAutoDriveSpeedFactor = 0.5

# Operator Interface
kDriverControllerPort = 0
