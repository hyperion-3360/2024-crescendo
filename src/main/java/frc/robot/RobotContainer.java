// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Shuffleboard3360;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Sequences;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.e_climberCheck;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Swerve m_swerveDrive = new Swerve();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  public static final Sequences m_sequence = new Sequences();
  public static final Shooter m_shooter = new Shooter();
  public static final Elevator m_elevator = new Elevator();
  public static final Trap m_trap = new Trap();
  public static final Blocker m_blocker = new Blocker();
  public static final Climber m_climber = new Climber();

  private final Shuffleboard3360 shuffleboard = Shuffleboard3360.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_swerveDrive.resetModulesToAbsolute();

    m_swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            m_swerveDrive,
            () -> -m_driverController.getRawAxis(translationAxis),
            () -> -m_driverController.getRawAxis(strafeAxis),
            () -> -m_driverController.getRawAxis(rotationAxis),
            () -> false));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.a().onTrue(Sequences.automaticTrapSequence(m_trap, m_shooter));

    // m_driverController.b().onTrue(Sequences.shootHigh( m_shooter, m_blocker));

    //  m_driverController.y().onTrue(Sequences.shootLow( m_shooter, m_blocker));

    //   m_coDriverController.x().whileTrue(Sequences.switchToIntakeMode( m_shooter,
    // m_blocker)).onFalse(m_shooter.stop());

    m_coDriverController.b().onTrue(m_climber.climberGoToSelectedLevel(e_climberCheck.BOTTOM));

    m_driverController.y().onTrue(m_climber.climberGoToSelectedLevel(e_climberCheck.TOP));
  }
}
