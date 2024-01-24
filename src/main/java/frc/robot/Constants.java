// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleConfig;

public final class Constants {

 

  public static class SubsystemConstants {
    // constants for climber, shooter, elevator, trap name example: kExamplePort, all ints if port
    public static final int kclimberRightId = 11; // climber
    public static final int kclimberLeftId = 12;
    public static final int kelevatorRightId = 9; // elevator 9
    public static final int kelevatorLeftId = 10; //10
    public static final int kShooterLeftMasterId = 0;
    public static final int kShooterRightMasterId = 0;
    public static final int kShooterLeftFollowerId = 0;
    public static final int kShooterRightFollowerId = 0;
  }

  public static class ShooterConstants {

    public static final int kRightMasterid = 13;
    public static final int kRightFollowerid = 15;
    public static final int kLeftMasterid = 14;
    public static final int kLeftFollowerid = 16;

    public static final double kI = 0.0;
    public static final double kP = 0.0;
    public static final double kD = 0.0;

        
  }

  public static class ElevatorConstants {
    //TODO change the values to suitable ones using april tag
   public static final double kHighTarget = 4.2;
   public static final double kLowTarget = 2.2;
   public static final double kIntakeTarget = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kPigeonPort = 0;
  }

  public static class WCPSwerveModule {

    // public static final WCPSwerveModuleConfig[] kConfigs = {
    //   new WCPSwerveModuleConfig(2, 1, 0, 9650),
    //   new WCPSwerveModuleConfig(4, 3, 1, 20756.0),
    //   new WCPSwerveModuleConfig(6, 5, 2, 1072.0),
    //   new WCPSwerveModuleConfig(8, 7, 3, 13555.0)
    // };
    public static final Translation2d[] kLocations = {
      // these values will have to be verified/edited
      new Translation2d(0.3525, 0.275),
      new Translation2d(0.3525, -0.275),
      new Translation2d(-0.3525, -0.275),
      new Translation2d(-0.3525, 0.275)
    };

    public static final double kAnalogToDeg = 360.0 / 28000;
    public static final double kDegToAnalog = 1.0 / kAnalogToDeg;

    public static final double kTickToMeter = 2.1 / 102260;
    public static final double kTickToMeterPerS = 10.0 * kTickToMeter;

    public static final double kMeterPerSToTick = 1.0 / kTickToMeterPerS;

    public static final double kTurnKp = 0.07; // was .15
    public static final double kTurnKi = 0.0;
    public static final double kTurnKd = 0.05;
    public static final double kTurnIZone = 0.0;

    public static final double kDriveKp = 0.02;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.08;
    public static final double kDriveKf = 0.04625;
    public static final double kDriveIZone = 0.0;
  }

}
