"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.system.plant import DCMotor

from util.keyorganization import OptionalValueKeys

# Basic units
kInchesPerFoot = 12
"""inches / foot"""

kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""

kMetersPerFoot = kMetersPerInch * kInchesPerFoot
"""meters / foot"""

kRadiansPerRevolution = 2 * math.pi
"""radians / revolution"""

kDegeersPerRevolution = 360
"""degrees / revolution"""

kRadiansPerDegree = kRadiansPerRevolution / kDegeersPerRevolution
"""radians / degree"""

kMillisecondsPerSecond = 1000 / 1
"""milliseconds / second"""

kSecondsPerMinute = 60 / 1
"""seconds / minute"""

# Debug parameters
kPrintFrequency = 2
""" 1 / second"""

kPrintPeriod = 1 / kPrintFrequency
"""seconds"""

# Field Physical parameters
kFieldLength = 52.5 * kMetersPerFoot
"""meters"""

kFieldWidth = 27 * kMetersPerFoot
"""meters"""

# Robot Physical parameters
kRobotWidth = 28 * kMetersPerInch
"""meters"""

kRobotLength = 28 * kMetersPerInch
"""meters"""

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

kFrontLeftWheelPosition = Translation2d(
    kHalfSwerveModuleCenterToCenterSideDistance,
    kHalfSwerveModuleCenterToCenterSideDistance,
)
"""[meters, meters]"""

kFrontRightWheelPosition = Translation2d(
    kHalfSwerveModuleCenterToCenterSideDistance,
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
)
"""[meters, meters]"""

kBackLeftWheelPosition = Translation2d(
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
    kHalfSwerveModuleCenterToCenterSideDistance,
)
"""[meters, meters]"""

kBackRightWheelPosition = Translation2d(
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
    -1 * kHalfSwerveModuleCenterToCenterSideDistance,
)
"""[meters, meters]"""

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

kMinWheelLinearVelocity = 0.002
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

kMaxWheelLinearAcceleration = kMaxWheelLinearVelocity / 1
"""meters / second^2"""

kMaxForwardLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxSidewaysLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxRotationAngularAcceleration = kMaxRotationAngularVelocity / 0.5
"""radians / second^2"""

kFrontLeftModuleName = "front_left"
kFrontRightModuleName = "front_right"
kBackLeftModuleName = "back_left"
kBackRightModuleName = "back_right"

kLimelightMountingOffset = Translation2d(0.0, 0.0)
kLimelightVerticalOffset = 25.833857 * kMetersPerInch  # derived from cad
"""meters"""
kLimelightVerticalAngleOffset = Rotation2d.fromDegrees(44.153)  # derived from cad
kTrackerPanAngleKey = "tracker/pan_angle"
kLimelightTrackerModuleName = "limelight_target_tracker"

# Limelight
kLimelightTargetInvalidValue = 0.0
kLimelightTargetValidValue = 1.0
kLimelightMinHorizontalFoV = Rotation2d.fromDegrees(-29.8)
kLimelightMaxHorizontalFoV = Rotation2d.fromDegrees(29.8)
kLimelightMinVerticalFoV = Rotation2d.fromDegrees(-22.85)
kLimelightMaxVerticalFoV = Rotation2d.fromDegrees(22.85)
kLimelightNetworkTableName = "limelight"
kLimelightTargetValidKey = "tv"
kLimelightTargetHorizontalAngleKey = "tx"
kLimelightTargetVerticalAngleKey = "ty"


# Limelight (cargo)
kLimelightCargoNetworkTableName = "limelight-cargo"

# CANivore
kCANivoreName = "canivore"

# Motors
kFrontLeftDriveMotorId = 10
kFrontLeftSteerMotorId = 11
kFrontRightDriveMotorId = 12
kFrontRightSteerMotorId = 13
kBackLeftDriveMotorId = 14
kBackLeftSteerMotorId = 15
kBackRightDriveMotorId = 16
kBackRightSteerMotorId = 17
kIntakeMotorId = 18
kIntakeMotorName = "IntakeMotor"
kIntakeMotorInverted = False
kIndexerMotorId = 19
kIndexerMotorName = "IndexerMotor"
kStagingMotorId = 20
kStagingMotorName = "StagingMotor"

# Encoders
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
kDrivePGain = 0.12
kDriveIGain = 0.0
kDriveDGain = 0.0

kSteerPIDSlot = 0
kSteerPGain = 0.6
kSteerIGain = 0.0
kSteerDGain = 12.0

kFrontLeftDriveInverted = False
kFrontRightDriveInverted = True
kBackLeftDriveInverted = False
kBackRightDriveInverted = True

kFrontLeftSteerInverted = False
kFrontRightSteerInverted = False
kBackLeftSteerInverted = False
kBackRightSteerInverted = False

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
kFrontLeftAbsoluteEncoderOffset = 0.264
"""degrees"""

kFrontRightAbsoluteEncoderOffset = 0.879
"""degrees"""

kBackLeftAbsoluteEncoderOffset = 179.736
"""degrees"""

kBackRightAbsoluteEncoderOffset = 0.352
"""degrees"""

kRobotPoseArrayKeys = OptionalValueKeys("RobotPoseArray")

kRobotVisionPoseWeight = 0.04  # 4% vision data

kDriveVelocityKeys = "robotVelocity"
kRobotUpdatePeriod = 1 / 50
"""seconds"""

# Vision parameters
kTargetAngleRelativeToRobotKeys = OptionalValueKeys("TargetAngleRelativeToRobot")
kTargetDistanceRelativeToRobotKeys = OptionalValueKeys("TargetDistanceRelativeToRobot")
kTargetFacingAngleRelativeToRobotKeys = OptionalValueKeys(
    "TargetFacingAngleRelativeToRobot"
)
kTargetPoseArrayKeys = OptionalValueKeys("TargetPoseArray")
kRobotVisionPoseArrayKeys = OptionalValueKeys("VisionRobotPose")

kTargetName = "Target"


kBallAngleRelativeToRobotKeys = OptionalValueKeys("BallAngleRelativeToRobot")
kBallDistanceRelativeToRobotKeys = OptionalValueKeys("BallDistanceRelativeToRobot")


# Autonomous
kAutoDriveDistance = -8 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5
"""dimensionless"""

kAutoWaitDuration = 1
"""seconds"""

kAutoTargetOffset = Translation2d(2, 0)
"""[meters, meters]"""

kAuto5BallFilename = "5ba"

# Target relative drive
kTargetRelativeDriveAnglePGain = 1
kTargetRelativeDriveAngleIGain = 0
kTargetRelativeDriveAngleDGain = 0

kRotationPGain = 0.8
kRotationIGain = 0
kRotationDGain = 0

# Drive to Target
kDriveToTargetDistancePGain = 0.5
kDriveToTargetDistanceIGain = 0
kDriveToTargetDistanceDGain = 0

kDriveToTargetAnglePGain = 0.5
kDriveToTargetAngleIGain = 0
kDriveToTargetAngleDGain = 0

kDriveToTargetDistanceTolerance = 10 / kCentimetersPerMeter
"""meters"""

kDriveToTargetLinearVelocityTolerance = 1 / kCentimetersPerMeter / 1
"""meters / second"""

kDriveToTargetAngleTolerance = 5 * kRadiansPerDegree
"""radians"""

kDriveToTargetAngularVelocityTolerance = 5 * kRadiansPerDegree / 1
"""radians / second"""

# Trajectory Following
kTrajectoryPositionPGain = 3.0
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 1.5
kTrajectoryAngleIGain = 0
kTrajectoryAngleDGain = 0

# Operator Interface
kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""

kControllerMappingFilename = "ControlScheme.json"

kChassisRotationXAxisName = "chassisXRotation"
kChassisRotationYAxisName = "chassisYRotation"
kChassisForwardsBackwardsAxisName = "chassisForwardsBackwards"
kChassisSideToSideAxisName = "chassisSideToSide"

kFieldRelativeCoordinateModeControlButtonName = "fieldRelativeCoordinateModeControl"
kResetGyroButtonName = "resetGyro"
kTargetRelativeCoordinateModeControlButtonName = "targetRelativeCoordinateModeControl"
kDriveToTargetControlButtonName = "driveToTargetControl"
kDeployIntakeButtonName = "deployIntake"
kReverseBallPathName = "reverseBallPath"
kXboxTriggerActivationThreshold = 0.5

# Simulation Parameters
kSimTargetName = "SimTarget"
kSimDefaultTargetLocation = Pose2d(
    kFieldLength / 2, kFieldWidth / 2, 180 * kRadiansPerDegree
)
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(kFieldLength / 2, kFieldWidth / 2, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in
kSimBallName = "SimBall"
kSimDefaultBallLocation = Pose2d(kFieldLength / 4, kFieldWidth / 2, 0)

"""meters"""

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimTargetPoseArrayKey = "SimTargetPoseArray"
kSimBallPoseArrayKey = "SimBallPoseArray"
kSimTargetHeightKey = "SimTargetHeight"
kSimTargetTrackingModuleName = "sim_target_tracker"
kSimTargetUpperHubRadius = 2

kSimFrontLeftDriveMotorPort = 0
kSimFrontLeftSteerMotorPort = 1
kSimFrontRightDriveMotorPort = 2
kSimFrontRightSteerMotorPort = 3
kSimBackLeftDriveMotorPort = 4
kSimBackLeftSteerMotorPort = 5
kSimBackRightDriveMotorPort = 6
kSimBackRightSteerMotorPort = 7
kSimIntakeMotorPort = 8
kSimStagingMotorPort = 18
kSimIndexerMotorPort = 19


kSimFrontLeftDriveEncoderPorts = (0, 1)
kSimFrontLeftSteerEncoderPorts = (2, 3)
kSimFrontRightDriveEncoderPorts = (4, 5)
kSimFrontRightSteerEncoderPorts = (6, 7)
kSimBackLeftDriveEncoderPorts = (8, 9)
kSimBackLeftSteerEncoderPorts = (10, 11)
kSimBackRightDriveEncoderPorts = (12, 13)
kSimBackRightSteerEncoderPorts = (14, 15)

# vision system / camera parameters
kCameraPanServoPWMChannel = 0
"""direct port number on the RoboRIO itself"""
kCameraSimPanServoPWMChannel = 9

kCameraTiltServoPWMChannel = 1
"""direct port number on the RoboRIO itself"""
kCameraSimTiltServoPWMChannel = 8

kCameraPanInverted = True
kCameraTiltInverted = False

kCameraServoMaxAngle = 60 * kRadiansPerDegree
"""radians"""

kCameraServoRotationNumberKey = "CameraServoRotation"

kCameraSimServoObjectName = "Camera Servo"

kCameraServoPGain = 0.15
kCameraServoIGain = 0.0
kCameraServoDGain = 0.0

# Climber Constants
kLeftClimberMotorCanID = 24
kRightClimberMotorCanID = 25
kSimRightClimberMotorID = 16
kSimLeftClimberMotorID = 17
kLeftClimberBrakePCMID = 0
kRightClimberBrakePCMID = 1
kLeftClimberInverted = True
kMoveLeftClimberToMiddleRungCapturePositionName = (
    "moveLeftClimberToMiddleRungCapturePosition"
)
kMoveRightClimberToMiddleRungCapturePositionName = (
    "moveRightClimberToMiddleRungCapturePosition"
)
kMoveBothClimbersToMiddleRungCapturePositionName = (
    "moveBothClimbersToMiddleRungCapturePosition"
)
kMoveLeftClimberToMiddleRungHangPositionName = "moveLeftClimberToMiddleRungHangPosition"
kMoveRightClimberToMiddleRungHangPositionName = (
    "moveRightClimberToMiddleRungHangPosition"
)
kMoveBothClimbersToMiddleRungHangPositionName = (
    "moveBothClimbersToMiddleRungHangPosition"
)
kHoldBothClimbersPositionName = "holdBothClimbersPosition"
kRightClimberMotorName = "climber_right"
kLeftClimberMotorName = "climber_left"
kPivotRightClimberToTilted = "pivotRightClimberToTilted"
kPivotLeftClimberToTilted = "pivotLeftClimberToTilted"
kPivotRightClimberToVertical = "pivotRightClimberToVertical"
kPivotLeftClimberToVertical = "pivotLeftClimberToVertical"
kLeftClimberEncoderTicksKey = "Left Encoder"
kRightClimberEncoderTicksKey = "Right Encoder"
kRightClimberPivotSolenoidForwardActuationID = 2
kLeftClimberPivotSolenoidForwardActuationID = 3
kLeftClimberPivotSolenoidBackwardActuationID = 12
kRightClimberPivotSolenoidBackwardActuationID = 13
kClimberMotorPGain = 0.025
kClimberMiddleRungCapturePosition = 344502
kClimberMiddleRungHangPosition = 250502
kClimberHangingPosition = 95102  # Not currently used
kClimberRetractionPositionThreshold = 3500
kClimberExtensionPositionThreshold = 2000

# shooter parameters
kHoodMotorId = 21
kTurretMotorId = 22
kShootingMotorId = 23

kSimTurretMotorPort = 9
kSimShootingMotorPort = 15
kSimHoodMotorPort = 16

kSimTurretMinimumLimitSwitchPort = 0
kSimTurretMaximumLimitSwitchPort = 1
kSimHoodMinimumSwitchPort = 2
kSimHoodMaximumSwitchPort = 3

kShootingPIDSlot = 0
kShootingPGain = 0.7
kShootingIGain = 0
kShootingDGain = 0
kShootingMotorInverted = True

kShootingMappingFunction = (
    lambda x: 16.41 * math.pow(math.e, 0.3662 * x) + 532
)  # derived from testing at multiple points, distance is input variable and exponential curve of best fit

kTurretPIDSlot = 0
kTurretPGain = 0.06
kTurretIGain = 0
kTurretDGain = 0
kTurretMotorInverted = False

kTurretGearRatio = (1 / 5) * (24 / 200)

kHoodPIDSlot = 0
kHoodPGain = 0.15
kHoodIGain = 0
kHoodDGain = 0
kHoodMotorInverted = True

kHoodGearRatio = (1 / 5) * (13 / 360)

kHoodMappingFunction = (
    lambda x: 11.04 * math.pow(math.e, 0.1004 * x) + -11.06
)  # see above for derivation method

kTurretMotorName = "shooting_turret"
kShootingMotorName = "shooting_shooting"
kHoodMotorName = "shooting_hood"

kShootingWheelSpeedKey = "shooting/wheelSpeed"
kShootingHoodAngleKey = "shooting/hoodAngle"
kShootingTurretAngleKey = "shooting/turretAngle"
kShootingManualModeKey = "shooting/manualMode"
kShootingFlywheelOnTargetKey = "shooting/wheelOnTarget"
kShootingHoodOnTargetKey = "shooting/hoodOnTarget"
kShootingTurretOnTargetKey = "shooting/turretOnTarget"

kHoodStartingAngle = 0

kTurretMaximumAngle = Rotation2d.fromDegrees(160)
kTurretMinimumAngle = Rotation2d.fromDegrees(-160)
kTurretSoftLimitBuffer = Rotation2d.fromDegrees(160 - 30)

kTurretRelativeForwardAngle = Rotation2d.fromDegrees(0)
kTurretOffsetFromRobotAngle = Rotation2d.fromDegrees(180)  # shooter 0 is robot 180

kHoodMaximum = Rotation2d.fromDegrees(18)
kHoodMinimum = Rotation2d.fromDegrees(-5)
kHoodSoftLimitBuffer = Rotation2d.fromDegrees(0.5)

kTurretAngleTolerence = Rotation2d.fromDegrees(2)
kHoodAngleTolerence = Rotation2d.fromDegrees(1)
kWheelSpeedTolerence = 20

kOffsetDistanceRange = 1
"""meters"""

kOffsetAngleRange = Rotation2d.fromDegrees(3)

kMotorBaseKey = "motors"
kPredictiveAimGain = 0.1

# Fender Shot
kFenderHoodAngle = Rotation2d.fromDegrees(0)
kFenderWheelSpeed = 600
"""rotations / minute """

# Intake Camera
kIntakeCameraTiltAngle = Rotation2d.fromDegrees(90 - 25)
kIntakeCameraHeightInMeters = 0.5
kIntakeCameraCenterOffsetInMeters = 0.2
kIsIntakeCameraCentered = False

kIntakeSpeed = 500
"""rpm"""
kStagingSpeed = 800
"""rpm"""
kIndexerSpeed = 800
"""rpm"""

kIntakeSolenoidChannelId = 4

kSimIntakeCameraObjectName = "Intake Camera"

# Intake auto intake constants
kDriveToBallPGain = 0.5
kDriveToBallIGain = 0
kDriveToBallDGain = 0.2
kAutoBallIntakeName = "autoBallIntake"

kShootBallButtonName = "shootBall"
kFenderShotButtonName = "fenderShot"
kManualModeToggleButtonName = "toggleManualModeShooter"
kTurretAngleOffsetAxisName = "turretOffset"
kShootingDistanceOffsetAxisName = "distanceOffset"


kIntakeRunningKey = "intake/running"
kIntakeReversedKey = "intake/reversed"

kIntakeSystemStateKey = "intakeState"
kIndexerSystemStateKey = "indexerState"
# Names are stored further up, about line 335

# Intake Sensors
kForwardSensorIndexer = True
kForwardSensorStaging = False

kSimIndexerSensorId = 16
kSimStagingSensorId = 17
