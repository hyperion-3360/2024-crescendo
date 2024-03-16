// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.PixelFormat;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.Sequences;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
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
  private final Trap m_trap = new Trap();
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private final Climber m_climber = new Climber();
  public static final Elevator m_elevator = new Elevator();
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
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  private final double kJoystickDeadband = 0.1;

  UsbCamera m_camera1, m_camera2;
  UsbCamera m_currentCam;
  VideoSink m_videoServer;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_camera1 = CameraServer.startAutomaticCapture(Constants.Camera.kCameraIntake);
    m_camera1.setVideoMode(
        PixelFormat.kMJPEG,
        Constants.Camera.kWidth,
        Constants.Camera.kHeight,
        Constants.Camera.kFPS);

    m_camera2 = CameraServer.startAutomaticCapture(Constants.Camera.kCameraDriver);
    m_camera2.setVideoMode(
        PixelFormat.kMJPEG,
        Constants.Camera.kWidth,
        Constants.Camera.kHeight,
        Constants.Camera.kFPS);

    m_videoServer = CameraServer.getServer();

    m_camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    m_camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    m_videoServer.setSource(m_camera2);
    m_currentCam = m_camera2;

    m_swerveDrive.resetModulesToAbsolute();

    m_swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            m_swerveDrive,
            () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
            () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
            () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
            () -> true));

    m_climber.setDefaultCommand(m_climber.run(() -> m_climber.setSpeed()));
    m_shooter.setDefaultCommand(m_shooter.stop());
    m_trap.setDefaultCommand(m_trap.setZero().unless(() -> m_trap.setZero));

    NamedCommands.registerCommand("shootHigh", AutoCommands.autoShoot(m_elevator, m_shooter));
    NamedCommands.registerCommand("intake", m_shooter.intake().withTimeout(3));
    NamedCommands.registerCommand("farShootA", AutoCommands.autoFarShoot1(m_elevator, m_shooter));
    NamedCommands.registerCommand("farShootB", AutoCommands.autoFarShoot2(m_elevator, m_shooter));
    NamedCommands.registerCommand("farShootC", AutoCommands.autoFarShoot3(m_elevator, m_shooter));
    NamedCommands.registerCommand("farShootD", AutoCommands.autoFarShoot4(m_elevator, m_shooter));
    NamedCommands.registerCommand("wait", new WaitCommand(1));

    configureBindings();

    Autos.setShuffleboardOptions();
  }

  /** Configure Joystick bindings for manually controlling and debugging the Trap arm */
  public void configureTrapDebugBindings() {

    // // map joystick POV primary direction to each joint of the arm
    // List<Pair<Trap.Joint, Trigger>> jointMap = new ArrayList<Pair<Trap.Joint, Trigger>>();
    // jointMap.add(new Pair<Trap.Joint, Trigger>(Trap.Joint.SHOULDER, m_driverController.povUp()));
    // jointMap.add(new Pair<Trap.Joint, Trigger>(Trap.Joint.ELBOW, m_driverController.povRight()));
    // jointMap.add(new Pair<Trap.Joint, Trigger>(Trap.Joint.WRIST, m_driverController.povDown()));
    // jointMap.add(new Pair<Trap.Joint, Trigger>(Trap.Joint.FINGER, m_driverController.povLeft()));

    // spotless:off
    /**
     * using the POV of the controller
     * 
     *       SHOULDER       Y -> DECREASE ANGLE by 1 degree
     *           x
     *           x
     * FINGER xxx xxx ELBOW X -> INCREASE ANGLE by 1 degree
     *           x
     *           x
     *         WRIST
     */
    // spotless:on
    // for (var joint_pair : jointMap) {
    //   m_driverController
    //       .x()
    //       .and(joint_pair.getSecond())
    //       .whileTrue(m_trap.manualControl(joint_pair.getFirst(), true));
    //   m_driverController
    //       .y()
    //       .and(joint_pair.getSecond())
    //       .whileTrue(m_trap.manualControl(joint_pair.getFirst(), false));
    // }
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

    // configureTrapDebugBindings();

    // m_coDriverController.povDown().onTrue(Sequences.trapShoot(m_shooter, m_trap));
    // m_coDriverController.povUp().onTrue(Sequences.trapScore(m_trap));
    // m_coDriverController
    //     .povLeft()
    //     .onTrue(Sequences.climbElevatorNote(m_elevator, m_shooter, m_trap));

    SmartDashboard.putData(m_trap);
    // m_coDriverController.a().onTrue(m_trap.shoulderDecrease());
    // m_coDriverController.b().onTrue(m_trap.shoulderIncrease());
    // m_coDriverController.x().onTrue(m_trap.elbowIncrease());
    // m_coDriverController.y().onTrue(m_trap.elbowDecrease());
    m_coDriverController.y().onTrue(m_trap.grabPosition());
    m_coDriverController.x().onTrue(Sequences.trapGetNote(m_shooter, m_trap));
    // m_coDriverController.b().onTrue(m_trap.storeNote());
    // m_driverController.a().onTrue(m_trap.temporaryCommand());
    // m_coDriverController.b().onTrue(m_trap.shoulderIncrease());
    // m_coDriverController.x().onTrue(m_trap.elbowIncrease());

    // m_coDriverController.povRight().onTrue(Sequences.climbElevator(m_elevator, m_shooter));
    // m_coDriverController.y().onTrue(Sequences.elevatorHigh(m_elevator, m_shooter, m_led));
    // m_coDriverController.a().onTrue(Sequences.elevatorLow(m_elevator, m_shooter, m_led));
    // m_coDriverController
    //     .leftBumper()
    //     .onTrue(Sequences.elevatorFarHighFromClimb(m_elevator, m_shooter, m_led));
    // m_coDriverController
    //     .rightBumper()
    //     .onTrue(Sequences.elevatorFarHighFromAmp(m_elevator, m_shooter, m_led));

    // m_coDriverController.b().onTrue(Sequences.shoot(m_shooter, m_elevator, m_led));

    // m_coDriverController.x().onTrue(m_elevator.extendTheElevator(elevatorHeight.INTAKE));

    // m_driverController
    //     .a()
    //     .toggleOnTrue(Sequences.intakeSequence(m_shooter, m_led, m_driverController));
    // m_driverController
    //     .start()
    //     .and(m_driverController.back())
    //     .onTrue(Sequences.blockShooterGears(m_shooter, m_led, m_driverController));

    // m_driverController
    //     .b()
    //     .toggleOnTrue(m_shooter.vomit().finallyDo(() -> m_led.setState(State.IDLE)));

    // m_driverController
    //     .y()
    //     .toggleOnTrue(m_shooter.eject().finallyDo(() -> m_led.setState(State.IDLE)));

    // m_driverController.x().onTrue(changeCameraPerspective());
  }

  public Command changeCameraPerspective() {
    return Commands.runOnce(
        () -> {
          m_currentCam = m_currentCam == m_camera1 ? m_camera2 : m_camera1;
          m_videoServer.setSource(m_currentCam);
        });
  }

  public Command getAutonomousCommand() {
    m_swerveDrive.setPose(
        PathPlannerAuto.getStaringPoseFromAutoFile(Autos.getSelectedOption().toString()));
    return new PathPlannerAuto(Autos.getSelectedOption().toString());
  }
}
