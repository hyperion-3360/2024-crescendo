// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

/** Defines required methods to create swerve modules. */
public interface SwerveModuleFactory {

  /**
   * Create all configured swerve module.
   *
   * @return Swerve module array.
   */
  SwerveModule[] createModules();

  /**
   * Gets the location of all modules in the robot chassis.
   *
   * @return Module location array.
   */
  Translation2d[] getLocations();
}
