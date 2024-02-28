package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // different speed possibilities
  public enum levelSpeed {
    HIGH,
    FAR_HIGH,
    LOW,
    INTAKE,
    TRAP,
    STOP,
    VOMIT,
    EJECT,
    CLIMB
  }

  private static double highSpeed = 0.85; // need to add perk to adjust speed according to distance
  private static double farHighSpeed = 1.0;
  private static double lowSpeed = 0.5; // requires testing
  private static double intakeSpeed = 0.4;
  private static double trapSpeed = 0.2; // requires testing
  private static double stopSpeed = 0;
  private static double climbSpeed = 0.3;
  private static double vomitSpeed = -0.5;
  private static double rampRate = 1; // to be tuned according to battery and time consumption

  // declaring speed member
  private double m_speed = 0;
  private double m_prev_speed = m_speed;
  private levelSpeed m_targetLevel = levelSpeed.STOP;

  // blocker constants
  private final double kIntakeHookAngleOpen = 5.0;
  private final double kIntakeHookAngleClose = 31;

  // declaring motors for the shooter
  private CANSparkMax m_leftMaster =
      new CANSparkMax(Constants.ShooterConstants.kLeftMasterId, MotorType.kBrushless);
  private CANSparkMax m_rightMaster =
      new CANSparkMax(Constants.ShooterConstants.kRightMasterId, MotorType.kBrushless);
  private CANSparkMax m_leftFollower =
      new CANSparkMax(Constants.ShooterConstants.kLeftFollowerId, MotorType.kBrushless);
  private CANSparkMax m_rightFollower =
      new CANSparkMax(Constants.ShooterConstants.kRightFollowerId, MotorType.kBrushless);

  // infrared sensor for intake
  private DigitalInput m_infraredSensor =
      new DigitalInput(Constants.ShooterConstants.kInfraredSensorId);

  // creating the blocker servo
  private Servo m_blocker = new Servo(Constants.ShooterConstants.kservoBlockerId);

  public Shooter() {

    // configs
    m_leftMaster.restoreFactoryDefaults();
    m_rightMaster.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();

    m_leftMaster.setInverted(true);
    m_leftFollower.setInverted(true);

    m_leftMaster.setIdleMode(IdleMode.kBrake);
    m_leftFollower.setIdleMode(IdleMode.kBrake);
    m_rightMaster.setIdleMode(IdleMode.kBrake);
    m_rightFollower.setIdleMode(IdleMode.kBrake);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    // the openLoopRampRate is a feature from spark max that caps the maximum acceleration from 0 to
    // +/- 1 in seconds
    m_rightMaster.setOpenLoopRampRate(rampRate);
    m_leftMaster.setOpenLoopRampRate(rampRate);
    m_rightFollower.setOpenLoopRampRate(rampRate);
    m_leftFollower.setOpenLoopRampRate(rampRate);

    m_rightMaster.burnFlash();
    m_leftFollower.burnFlash();
    m_rightFollower.burnFlash();
    m_leftMaster.burnFlash();

    // setting the blocker to closed on boot
    m_blocker.setAngle(kIntakeHookAngleClose);
  }

  @Override
  public void periodic() {
    // setting speed to motors
    SmartDashboard.putBoolean("Has Note?", hasNote());
  }

  // switch case for different speeds according to the level
  private void setSpeedFor(levelSpeed shoot) {
    switch (shoot) {
      case LOW:
        m_speed = lowSpeed;
        break;

      case HIGH:
        m_speed = highSpeed;
        break;
      case FAR_HIGH:
        m_speed = farHighSpeed;
        break;
      case STOP:
        m_speed = stopSpeed;
        break;

      case TRAP:
        m_speed = trapSpeed;
        break;

      case INTAKE:
        m_speed = intakeSpeed;
        break;

      case VOMIT:
        m_speed = vomitSpeed;
        break;

      case EJECT:
        m_speed = -vomitSpeed;
        break;
      case CLIMB:
        m_speed = climbSpeed;
        break;
    }
    if (m_speed != m_prev_speed) {
      m_prev_speed = m_speed;
      m_leftMaster.set(m_speed);
      m_rightMaster.set(m_speed);
    }
  }

  // stop the motors
  public Command stop() {
    return this.runOnce(() -> this.setSpeedFor(levelSpeed.STOP));
  }

  // shoot to desired level
  public Command shootTo() {
    return Commands.sequence(
        this.hookIntake(),
        this.setSpeedWithTarget(),
        new WaitCommand(1),
        this.hookRelease(),
        new WaitCommand(0.7),
        this.stop(),
        this.hookIntake());
  }

  // setting the target level
  public Command setTargetLevel(levelSpeed level) {
    return this.runOnce(() -> m_targetLevel = level);
  }

  // set the speed according to the target
  public Command setSpeedWithTarget() {
    return this.runOnce(() -> this.setSpeedFor(m_targetLevel));
  }

  // set hook to intake mode, then set target + speed until has note
  public Command intake() {
    return Commands.sequence(
        this.hookIntake(),
        this.runOnce(
            () -> {
              this.setSpeedFor(levelSpeed.INTAKE);
            }),
        new WaitUntilCommand(this::hasNote),
        this.stop());
  }

  // infrared sensor to see if we have a note
  public boolean hasNote() {
    return !m_infraredSensor.get();
  }

  // set down the hook
  public Command hookIntake() {
    return this.runOnce(() -> m_blocker.setAngle(kIntakeHookAngleClose));
  }

  // set up the hook
  public Command hookRelease() {
    return this.runOnce(() -> m_blocker.setAngle(kIntakeHookAngleOpen));
  }

  // EMERGENCY command in case of intake anomaly
  public Command vomit() {
    return Commands.sequence(
        setTargetLevel(levelSpeed.VOMIT),
        setSpeedWithTarget(),
        new WaitCommand(4),
        setTargetLevel(levelSpeed.STOP),
        setSpeedWithTarget());
  }

  public Command eject() {
    return Commands.sequence(
        hookRelease(),
        setTargetLevel(levelSpeed.EJECT),
        setSpeedWithTarget(),
        new WaitCommand(4),
        setTargetLevel(levelSpeed.STOP),
        setSpeedWithTarget(),
        hookIntake());
  }

  public Command holdSpeed(levelSpeed level) {
    return this.run(() -> this.setSpeedFor(level));
  }
}
