// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Defines required methods for a functionnal swerve module */
public interface SwerveModule {
  /**
   * Gets the current state of the module
   *
   * @return Current module state
   */
  public SwerveModuleState getState();

  /**
   * Gets the current module position in term of angle and distance
   *
   * @return Current module position
   */
  public SwerveModulePosition getPosition();

  public SwerveModulePosition[] getPositions();

  /**
   * Sets the desired state for the module.
   *
   * <p>State will be optimized before being used as control loops setpoints.
   *
   * @param desiredState Desired module state
   */
  public void setDesiredState(SwerveModuleState desiredState);

  /** This method will be called automatically every scheduler loop. */
  public void periodic();
}
