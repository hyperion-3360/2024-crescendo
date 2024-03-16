// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final long kSecondsToMicroSeconds = 1000000;

  public static final double stickDeadband = 0.1;

  public static final class LED {
    public static final int kLEDredPWMPort = 0;
    public static final int kLEDgreenPort = 1;
    public static final int kLEDbluePort = 3;
    public static final int kLEDwhitePort = 2;
  }

  public static final class Camera {
    public static final int kWidth = 320;
    public static final int kHeight = 200;
    public static final int kFPS = 10;
    public static final int kCameraDriver = 1;
    public static final int kCameraIntake = 0;
  }

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.Falcon500(
            COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X1_10);

    // coming from datasheet using typical values
    // https://store.ctr-electronics.com/content/user-manual/Magnetic%20Encoder%20User%27s%20Guide.pdf
    public static final double kPwmPeriod = 1.0 / 244.0;
    public static final double dutyCycleMin = 1e-6 / kPwmPeriod;
    public static final double dutyCycleMax = 4096e-6 / kPwmPeriod;

    public static final int calibrationFreqSamples = 30;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(26);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front left
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // front right
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // back left
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); // back right

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32;
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5;

    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int magEncoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(155.2);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int magEncoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int magEncoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(14.3);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int magEncoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(312.3);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }
  }

  public static class SubsystemConstants {
    // constants for climber, shooter, elevator, trap name example: kExamplePort, all ints if port
    public static final int kclimberRightId = 11; //  climber
    public static final int kclimberLeftId = 12;
    public static final int kelevatorRightId = 9; // elevator
    public static final int kelevatorLeftId = 10;
  }

  public static class ClimberConstants {
    public static final double kTopTarget = -37;
    public static final double kstartPos = 8;
  }

  public static class ShooterConstants {
    public static final int kRightMasterId = 16;
    public static final int kLeftMasterId = 14;
    public static final int kLeftFollowerId = 13;
    public static final int kRightFollowerId = 15;
    public static final int kInfraredSensorId = 4;
    public static final int kShooterOutputIR = 5;
    public static final int kservoBlockerId = 5;
    public static final int kservoGearBlockerId = 4;
    public static double kMaxShootingDistance =
        4.0; // max shooting distance in meters allowed in AimLock
  }

  public static class ElevatorConstants {
    public static final double kDeadzone = 0.0035;
    public static final double kHighTarget = 68.8; // was negativ, changed to mec request
    public static final double kFarHighTarget = 68.8; // needs more fine tuning maybe
    public static final double kLowTarget = 65; // 61
    public static final double kIntakeTarget = 0.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final int kPigeonPort = 0;
  }

  public static class TrapConstants {
    public static final int kShoulderId = 18; // servo ports + limit switch
    public static final int kElbowId = 17;
    public static final int kServoWristId = 8;
    public static final int kServoFingerId = 9;
    public static final int kLimitSwitchId = 6;

    public static final double kFingerClosed = 180.0; // angle of finger servo when closed/opened
    public static final double kFingerOpened = 90.0;
    // good
    public static final double kShoulderSetZero = 0.41; // 0.41 SETZERO
    public static final double kElbowSetZero = 0.05;
    public static final double kWristSetZero = 155.0; // higher number = higher
    // good
    public static final double kShoulderGrabPosition = 0.16; // GRABPOSITION
    public static final double kElbowGrabPosition = 0.23;
    public static final double kWristGrabPosition = 200.0;

    // good
    public static final double kShoulderStoreNote = 0.41; // STORENOTE
    public static final double kElbowStoreNote = 0.05;
    public static final double kWristStoreNote = 155.0;

    public static final double kShoulderPushNote = 0.0; // PUSHNOTE
    public static final double kElbowPushNote = 0.0;
    public static final double kWristPushNote = 100.0;

    // Shoulder moves first to its most vertical position  // DUNKNOTE + PUSH
    public static final double kShoulderDunkNote = 0.0;
    // Elbow and wrist initial positions to stuck the note at an angle
    public static final double kElbowDunkNote = 0.0;
    public static final double kWristDunkNote = 50.0;
    // Try to drop the note into the trap by pushing down and moving up the wrist a bit
    public static final double kWristFinalPush = 5.0;
    public static final double kElbowFinalPush = 0.0;

    // in progress
    public static final double kShoulderDisable1 = 0.0; // PREPARETODISABLE1
    public static final double kElbowDisable1 = 0.0;
    public static final double kWristDisable1 = 165.0;
    // in progress
    public static final double kShoulderDisable2 = 70.0; // PREPARETODISABLE2
    public static final double kElbowDisable2 = 60.0;
    public static final double kWristDisable2 = 0.0;

    // sorry for all the delayed in prepare to climb but they are needed to avoid the arm from
    // hitting everywhere and they also make it less hard for servos to get from one position to
    // the other
    public static final double kShoulderPrepareToClimb = 0.0;
    public static final double kElbowPrepareToClimb = 0.0;
    public static final double kWristPrepareToClimb = 155.0;
  }

  public static class AutoConstants {
    public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            Swerve.maxSpeed,
            Math.sqrt(
                (Swerve.trackWidth / 2.0 * Swerve.trackWidth / 2.0)
                    + (Swerve.wheelBase / 2.0 * Swerve.wheelBase / 2.0)),
            new ReplanningConfig());
  }

  public static class VisionConstants {
    public static long kPositionCoalescingTime = 100 * 1000; // 100 ms in microseconds
    public static int kRedSpeakerTag = 4;
    public static int kBlueSpeakerTag = 7;
    public static int kSpeakerIndex[] = {kRedSpeakerTag, kBlueSpeakerTag};
    public static int kRedAmpTag = 5;
    public static int kBlueAmpTag = 6;
    public static int kAmpIndex[] = {kRedAmpTag, kBlueAmpTag};
    public static double kAmpRiseElevatorDistance = 0.5;
    public static double kAmpShootingDistance = 0.1;
  }
}
