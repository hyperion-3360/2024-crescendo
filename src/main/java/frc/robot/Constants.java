// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final double stickDeadband = 0.1;

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
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Coast;

    /* Module Specific Constants */

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int magEncoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(126.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int magEncoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(65.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int magEncoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.8);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, magEncoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int magEncoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(282.3);
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
    public static final int kShooterLeftMasterId = 0;
    public static final int kShooterRightMasterId = 0;
    public static final int kShooterLeftFollowerId = 0;
    public static final int kShooterRightFollowerId = 0;

    public static final int kRightMasterId = 16;
    public static final int kLeftMasterId = 14;
    public static final int kLeftFollowerId = 13;
    public static final int kRightFollowerId = 15;
  }

  public static class ElevatorConstants {
    public static final double kDeadzone = 0.0035;
    public static final double kHighTarget = 0.25;
    public static final double kLowTarget = 0.10;
    public static final double kIntakeTarget = 0.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final int kPigeonPort = 0;
  }

  public static class TrapConstants {

    public static final int kservoShoulderId = 9; // servo ports + limit switch
    public static final int kservoElbowId = 8;
    public static final int kservoWristId = 7;
    public static final int kservoFingerId = 6;
    public static final int kfingerlimitswitchId = 6;

    public static final double kfingerClosed = 180.0; // angle of finger servo when closed/opened
    public static final double kfingerOpened = 90.0;

    public static final double kangleShouldersetZero =
        7.0; // angles servos in SETZERO   //lower number = lower
    public static final double kangleElbowsetZero = 145.5; // higher number = lower
    public static final double kangleWristsetZero = 155.0; // higher number = lower

    public static final double kangleShouldergrabPosition = 70.0; // angles servos in GRABPOSITION
    public static final double kangleElbowgrabPosition = 105.0;
    public static final double kangleWristgrabPosition = 75.0;

    public static final double kangleShoulderscoreNote = 7.0; // angles servos in SCORENOTE
    public static final double kangleElbowscoreNote = 145.5;
    public static final double kangleWristscoreNote = 155.0;
    public static final double kangleFingerscoreNote = 155.0;
    public static final int kservoBlockerId = 5;
  }
}
