// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleConfig;

public final class Constants {

  public static class SubsystemConstants {
    // constants for climber, shooter, elevator, trap name example: kExamplePort, all ints if port
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kPigeonPort = 0;
  }

  public static class WCPSwerveModule {

    public static final WCPSwerveModuleConfig[] kConfigs = {
      new WCPSwerveModuleConfig(2, 1, 0, 9650),
      new WCPSwerveModuleConfig(4, 3, 1, 20756.0),
      new WCPSwerveModuleConfig(6, 5, 2, 1072.0),
      new WCPSwerveModuleConfig(8, 7, 3, 13555.0)
    };
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
