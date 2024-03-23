package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
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
    CLIMB,
    MAX,
  }

  private static double maxSpeed = 1.0;
  private static double highSpeed = 0.8; // need to add perk to adjust speed according to distance
  private static double farHighSpeed = 0.95;
  private static double lowSpeed = 0.5; // requires testing
  private static double intakeSpeed = 0.4;
  private static double trapSpeed = 0.17; // requires testing
  private static double stopSpeed = 0;
  private static double climbSpeed = 0.4;
  private static double vomitSpeed = -1;
  private static double rampRate = 1; // to be tuned according to battery and time consumption

  // declaring speed member
  private double m_speed = 0;
  private double m_prev_speed = m_speed;
  private levelSpeed m_targetLevel = levelSpeed.STOP;

  // blocker constants
  private final double kIntakeHookAngleOpen = 100.0;
  private final double kIntakeHookAngleClose = 156;

  // gear blocker constants
  private final double kGearBlocked = 0.0;
  private final double kGearBlockerRestPosition = 180.0;

  private boolean gearSetZero = false;

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
  private DigitalInput m_shotIR = new DigitalInput(Constants.ShooterConstants.kShooterOutputIR);

  private enum IntakeNoteStatus {
    IDLE,
    HAS_NOTE,
    FIRST_ARC,
    DOUGHNUT,
    SECOND_ARC,
    HAS_SHOT
  }

  private IntakeNoteStatus m_intakeNoteStatus = IntakeNoteStatus.IDLE;

  // creating the blocker servo
  private Servo m_blocker = new Servo(Constants.ShooterConstants.kservoBlockerId);
  // creating the gear blocker servo
  private Servo m_gearBlocker = new Servo(Constants.ShooterConstants.kservoGearBlockerId);

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
    m_gearBlocker.setAngle(180.0);
  }

  @Override
  public void periodic() {
    triggerIntakeNoteStatus();

    // setting speed to motors
    SmartDashboard.putBoolean("Has Note?", hasNote());
    // SmartDashboard.putBoolean("Has Shot?", hasShot());
    // SmartDashboard.putString("Intake status", m_intakeNoteStatus.toString());
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
      case MAX:
        m_speed = maxSpeed;
        break;
      case STOP:
        m_speed = stopSpeed;
        break;

      case TRAP:
        m_speed = trapSpeed * 13.0 / RobotController.getBatteryVoltage();
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
    return this.runOnce(() -> this.setSpeedFor(levelSpeed.STOP)).andThen(hookIntake());
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

  public void consumeShotStatus() {
    m_intakeNoteStatus = IntakeNoteStatus.IDLE;
  }

  public boolean hasShot() {
    return (m_intakeNoteStatus == IntakeNoteStatus.HAS_SHOT);
  }

  /***
   * State machine to detect when a note is shot out of the intake
   * The IR beams do not have a high enough frequency to detect the
   * 2 half-circle arcs of the note at high speeds, so we have a max shot delay
   * Kinda working, but TODO: optimize this stuff (detect when note is shot)
   * The IR for the shot was recently moved and we are now constantly in state FIRST_ARC
   * when a note is intaked... Seems to still be working fine
   */
  private Double m_noteStatusTimer = null;

  private Double m_shootSequenceTimer = null;
  private double MAX_SHOT_DELAY = 1; // 1s max to execute the shoot sequence

  private boolean resetIfMaxShotDelay() {
    if (m_shootSequenceTimer != null
        && (Timer.getFPGATimestamp() - m_shootSequenceTimer) > MAX_SHOT_DELAY) {
      System.out.println("Resetting status for shot. Max delay passed");
      m_intakeNoteStatus = IntakeNoteStatus.HAS_SHOT;
      m_shootSequenceTimer = null;
      return true;
    }
    return false;
  }

  private void triggerIntakeNoteStatus() {
    switch (m_intakeNoteStatus) {
      case IDLE:
      case HAS_SHOT:
        if (!m_infraredSensor.get()) {
          m_intakeNoteStatus = IntakeNoteStatus.HAS_NOTE;
        }
        break;
      case HAS_NOTE:
        if (!m_shotIR.get()) {
          m_intakeNoteStatus = IntakeNoteStatus.FIRST_ARC;
        }
        break;
      case FIRST_ARC: // Wait until we detect the center of the note
        if (m_shotIR.get()) {
          m_shootSequenceTimer = Timer.getFPGATimestamp(); // Start the timer for the shoot sequence
          m_intakeNoteStatus = IntakeNoteStatus.DOUGHNUT;
        }
        break;
      case DOUGHNUT: // Detect second arc of the note to know that the shot is complete
        if (!resetIfMaxShotDelay() && !m_shotIR.get()) {
          m_intakeNoteStatus = IntakeNoteStatus.SECOND_ARC;
        }
        break;
      case SECOND_ARC: // Wait until the note has passed
        if (!resetIfMaxShotDelay() && m_shotIR.get()) {
          m_shootSequenceTimer = null;
          m_intakeNoteStatus = IntakeNoteStatus.HAS_SHOT;
        }
        break;
    }

    // Failsafe in case past a certain delay, both IR sensors do not detect a note, reset the status
    if (m_intakeNoteStatus != IntakeNoteStatus.IDLE && m_infraredSensor.get() && m_shotIR.get()) {
      if (m_noteStatusTimer == null) {
        m_noteStatusTimer = Timer.getFPGATimestamp();
      } else if (Timer.getFPGATimestamp() - m_noteStatusTimer > 2) { // 1s delay max...
        // System.out.println("Note status failsafe trigerred. Resetting status");
        m_intakeNoteStatus = IntakeNoteStatus.IDLE;
        m_noteStatusTimer = null;
      }
    } else if (m_noteStatusTimer != null) { // if the note is only passing through
      m_noteStatusTimer = null;
    }
  }

  // set down the hook
  public Command hookIntake() {
    return this.runOnce(() -> m_blocker.setAngle(kIntakeHookAngleClose));
  }

  // set the gear blocker hook into blocking position to prevent the gears from turning
  public Command gearBlockMode() {
    return this.runOnce(() -> m_gearBlocker.setAngle(kGearBlocked));
  }

  // set the gear blocker hook into rest postion so the gears aren't blocked during a match
  public Command gearBlockerRestMode() {
    return this.runOnce(() -> m_gearBlocker.setAngle(kGearBlockerRestPosition));
  }

  // set up the hook
  public Command hookRelease() {
    return this.runOnce(() -> m_blocker.setAngle(kIntakeHookAngleOpen));
  }

  public Command waitForShot() {
    return Commands.sequence(
        new WaitUntilCommand(this::hasShot), this.runOnce(this::consumeShotStatus));
  }

  // EMERGENCY command in case of intake anomaly
  public Command vomit() {
    return Commands.sequence(
        setTargetLevel(levelSpeed.VOMIT),
        setSpeedWithTarget(),
        new WaitCommand(4),
        this.runOnce(
            () -> {
              m_intakeNoteStatus = IntakeNoteStatus.IDLE;
            }),
        setTargetLevel(levelSpeed.STOP),
        setSpeedWithTarget());
  }

  public Command eject() {
    return Commands.sequence(
        hookRelease(),
        setTargetLevel(levelSpeed.EJECT),
        setSpeedWithTarget(),
        new WaitCommand(2),
        setTargetLevel(levelSpeed.STOP),
        setSpeedWithTarget(),
        hookIntake());
  }

  public Command holdSpeed(levelSpeed level) {
    return this.run(() -> this.setSpeedFor(level));
  }

  public Command shooterDefaultCommand() {
    return this.runOnce(() -> this.stop())
        .andThen(
            this.runOnce(
                    () -> gearBlockerRestMode().alongWith(this.runOnce(() -> gearSetZero = true)))
                .unless(() -> this.gearSetZero));
  }
}
