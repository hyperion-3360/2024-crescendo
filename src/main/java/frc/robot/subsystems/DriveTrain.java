// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleFactory;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DriveTrain extends SubsystemBase {

  // Subsystem parameters
  public static final double kMaxModuleSpeed = 4.0;

  public static final double kMaxSpeedX = 4.0;
  public static final double kMaxSpeedY = 4.0;
  public static final double kMaxSpeedRot = 360.0;

  public static final double kMaxAccTrans = 2.0 * kMaxModuleSpeed;
  public static final double kMaxAccRot = 4.0 * kMaxSpeedRot;

  public static final double kJoystickDeadband = 0.15;

  public static final double kHoloKP = 2.0;
  public static final double kHoloKI = 0.0;
  public static final double kHoloKD = 0.0;

  public static final double kRotKP = 8.0;
  public static final double kRotKI = 0.0;
  public static final double kRotKD = 0.0;

  public double filteredX = 0;
  public static final double XScoringPos = 2;
  public static final double minYScoringPos = 0.5;
  public static final double maxYScoringPos = 5;
  public static final double scoringGridIncrements = (maxYScoringPos - minYScoringPos) / 8;
  public double YScoringPos = 2.75;
  public boolean m_hadRecentVision = false;

  public static final int kPathServerPort = 5811;

  // Member objects
  private final SwerveModuleFactory m_moduleFactory = new WCPSwerveModuleFactory();
  private final SwerveModule[] m_modules = m_moduleFactory.createModules();
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_moduleFactory.getLocations());

  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(Constants.OperatorConstants.kPigeonPort);
  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          m_kinematics, m_gyro.getRotation2d(), this.getModulePositions(), new Pose2d());


  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kMaxAccTrans);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kMaxAccTrans);
  private final SlewRateLimiter m_zLimiter = new SlewRateLimiter(kMaxAccRot);

  private Field2d m_field = new Field2d();




  /** Creates a new DriveTrain. */
  public DriveTrain() {


    // Reset gyro on code startup (Required as odometry starts at 0)
    m_gyro.reset();
    // Run path planning server
    SmartDashboard.putData("field", m_field);
  }

  @Override
  public void periodic() {
    // Call module periodic

    for (final var module : m_modules) {
      module.periodic();
    }
  }
  /**
   * Gets the current position of all modules
   *
   * @return array of module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    var positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < positions.length; ++i) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Drives the robot in closed-loop velocity
   *
   * @param xSpeed Velocity along x (m/s)
   * @param ySpeed Velocity along y (m/s)
   * @param rotSpeed Rotation speed around x (deg/s)
   * @param fieldRelative Velocity are field relative
   */
  private void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    rotSpeed = Math.toRadians(rotSpeed);
    var moduleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rotSpeed, m_odometry.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxModuleSpeed);
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * Drives the robot in closed-loop velocity
   *
   * @param moduleStates Swerve modules states
   */
  public void drive(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < m_modules.length; ++i) {
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  /**
   * Gets the current pose of the robot from the state estimator
   *
   * @return Field relative robot pose
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Scales a joystick input to make it suitable as a velocity value
   *
   * @param valueSupplier Raw joystick value supplier (-1, 1)
   * @param maxValue Maximum value in physical units
   * @param rateLimiter Slew rate limiter to limit rate of change
   * @return Scaled joystick value (physical units)
   */
  private static double scaleJoystickInput(
      DoubleSupplier valueSupplier, double maxValue, SlewRateLimiter rateLimiter) {

    double rawValue = valueSupplier.getAsDouble();
    double value =
        Math.abs(rawValue) > kJoystickDeadband ? rawValue * rawValue * Math.signum(rawValue) : 0.0;

    return rateLimiter.calculate(value * maxValue);
  }

  /**
   * Command that drives using axis from the provided suppliers
   *
   * @param xSpeed Velocity along x (-1, 1) supplier
   * @param ySpeed Velocity along y (-1, 1) supplier
   * @param rotSpeed Rotation speed around x (-1, 1) supplier
   * @param fieldRelative Velocity are field relative supplier
   * @return run command that runs forever
   */
  public Command driveCommand(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rotSpeed,
      boolean fieldRelative) {

    return this.runOnce(
            () -> {
              this.m_xLimiter.reset(0.0);
              this.m_yLimiter.reset(0.0);
              this.m_zLimiter.reset(0.0);
            })
        .andThen(
            this.run(
                () ->
                    this.drive(
                        DriveTrain.scaleJoystickInput(xSpeed, kMaxSpeedX, m_xLimiter),
                        DriveTrain.scaleJoystickInput(ySpeed, kMaxSpeedY, m_yLimiter),
                        DriveTrain.scaleJoystickInput(rotSpeed, kMaxSpeedRot, m_zLimiter),
                        fieldRelative)));
  }
}


 

