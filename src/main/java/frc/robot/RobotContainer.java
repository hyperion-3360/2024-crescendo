// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Sequences;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;
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
  private final Trap m_trap = new Trap();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private final Climber m_climber = new Climber();
  private static final Elevator m_elevator = new Elevator();
  private static final Shooter m_shooter = new Shooter();
  private static final LEDs m_led = LEDs.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_coDriverController = new CommandXboxController(1);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2);

  private final double kJoystickDeadband = 0.1;

  // SendableChooser<Command> m_autoChooser;
  // ComplexWidget m_autoList;

  /***
   * conditionJoystick
   * Condition a joystick axis value given a slewrate limiter and deadband
   * @param axis axis to condition
   * @param limiter slewrate limiter (to smooth the rate of changed
   * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html)
   * @param deadband deadband to suppress noise around the 0 of a joystick axis
   * @see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/MathUtil.html#applyDeadband(double,double)
   * @return the conditioned value
   */
  private double conditionJoystick(int axis, SlewRateLimiter limiter, double deadband) {
    return -limiter.calculate(
        MathUtil.applyDeadband(m_driverController.getRawAxis(axis), deadband));
  }

  private ModeAuto m_autoHandler = new ModeAuto();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // create mode auto chooser and widget
    // m_autoChooser = new SendableChooser<>();

    // // example:
    // m_autoChooser.addOption("add name here", new WaitCommand(1));

    // creates a choosable list widget
    // m_autoList =
    //
    // Shuffleboard.getTab("Auto").add(m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    m_swerveDrive.resetModulesToAbsolute();

    m_swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            m_swerveDrive,
            () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
            () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
            () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
            () -> false));

    String shoot = "shoot hight";
    NamedCommands.registerCommand(shoot, highGoal());
    String shootlow = "shoot low";
    NamedCommands.registerCommand(shootlow, lowGoal());
    String take = "take";
    NamedCommands.registerCommand(take, takeNote());

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

    // m_driverController.a().onTrue(m_trap.setZero());
    // m_driverController.b().onTrue(m_trap.grabPosition());
    // m_driverController.x().onTrue(m_trap.scoreNote());
    m_coDriverController.y().onTrue(Sequences.elevatorHigh(m_elevator, m_shooter, m_led));
    m_coDriverController.a().onTrue(Sequences.elevatorLow(m_elevator, m_shooter, m_led));
    m_coDriverController.x().onTrue(m_elevator.extendTheElevator(elevatorHeight.INTAKE));
    // m_coDriverController.b().onTrue(Sequences.shoot(m_shooter, m_elevator, m_led));

    // control pos with triggers but idk if it works
    // m_coDriverController
    //     .leftTrigger()
    //     .whileTrue(m_climber.climberGoToSelectedLevel(climberPos.INITAL));
    // m_coDriverController
    //     .rightTrigger()
    //     .whileTrue(m_climber.climberGoToSelectedLevel(climberPos.TOP));

    // m_driverController.y().onTrue(m_shooter.intake());
    // m_coDriverController.a().onTrue(Sequences.trapShoot(m_shooter, m_trap));
  }

  public void autoInit() {
    // TODO Selectionner le mode auto du shuffleboard
    m_autoHandler.follow(ModeAuto.Mode.RED_AUTO1);
  }

  public Command highGoal() {
    return m_elevator
        .extendTheElevator(Elevator.elevatorHeight.HIGH)
        .andThen(m_shooter.setTargetLevel(levelSpeed.HIGH).andThen(m_shooter.shootTo()));
  }

  public Command lowGoal() {
    return m_elevator
        .extendTheElevator(Elevator.elevatorHeight.LOW)
        .andThen(m_shooter.setTargetLevel(levelSpeed.LOW).andThen(m_shooter.shootTo()));
  }

  public Command takeNote() {
    return m_elevator
        .extendTheElevator(Elevator.elevatorHeight.INTAKE)
        .andThen(
            () -> {
              m_shooter.intake();
            });
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
  }
}
