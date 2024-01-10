// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WCPSwerveModule;

/** Add your docs here. */
public class WCPSwerveModuleConfig {

  public final int m_turnMotorId;
  public final int m_driveMotorId;
  public final double m_analogZero;
  public final int m_magEncoderChannel;

  public WCPSwerveModuleConfig(
      int turnMotorId, int driveMotorId, int magEncoderChannel, double analogZero) {
    m_turnMotorId = turnMotorId;
    m_driveMotorId = driveMotorId;
    m_magEncoderChannel = magEncoderChannel;
    m_analogZero = analogZero;
  }
}
