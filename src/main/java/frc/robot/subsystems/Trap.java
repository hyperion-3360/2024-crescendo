package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.TimedServo;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  DoubleLogEntry m_shoulderThetaCurrentLog;
  DoubleLogEntry m_shoulderThetaComputedLog;

  DoubleLogEntry m_shoulderOmegaCurrentLog;
  DoubleLogEntry m_shoulderOmegaComputedLog;

  DoubleLogEntry m_shoulderVoltageLog;

  DoubleLogEntry m_elbowThetaCurrentLog;
  DoubleLogEntry m_elbowThetaComputedLog;

  DoubleLogEntry m_elbowOmegaComputedLog;
  DoubleLogEntry m_elbowOmegaCurrentLog;

  DoubleLogEntry m_elbowVoltageLog;

  // the vertical position is 0.1394 substracting a quarter or a turn to get 0 deg parallel to
  // ground in positive direction
  private final double m_zeroPosShoulder = 0.1394 - 0.25;

  private final double m_zeroPosElbow =
      0.4770; // horizontal zero in absolution position of shoulder

  private static final double kDt = 0.02;

  // shoulder joint control
  private final double kshoulderDeadZoneEnd = 0.35;
  private final double kshoulderDeadZoneBegin = 0.05;

  private final double kP = 80;
  private final double kI = 50;
  private final double kD = 2.0;
  private final double sKv = 1 / 19;
  private final double sKg = -2;

  private CANSparkMax m_shoulder =
      new CANSparkMax(Constants.TrapConstants.kShoulderId, MotorType.kBrushed);

  private final SparkAbsoluteEncoder m_shoulderEncoder =
      m_shoulder.getAbsoluteEncoder(Type.kDutyCycle);

  private final TrapezoidProfile m_shoulderProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 0.5));

  private TrapezoidProfile.State m_shoulderGoal;
  private TrapezoidProfile.State m_shoulderSetpoint;
  private PIDController m_shoulderPid = new PIDController(kP, kI, kD);
  private ArmFeedforward m_shoulderFF = new ArmFeedforward(0, sKg, sKv);
  private double m_shoulderVoltage;

  // elbow joint control
  private final double kelbowDeadZoneBegin = 0.05;
  private final double kelbowDeadZoneEnd = 0.50;

  private CANSparkMax m_elbow =
      new CANSparkMax(Constants.TrapConstants.kElbowId, MotorType.kBrushed);

  private final SparkAbsoluteEncoder m_elbowEncoder = m_elbow.getAbsoluteEncoder(Type.kDutyCycle);

  private final TrapezoidProfile m_elbowProfile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(0.3, 0.1));

  private TrapezoidProfile.State m_elbowGoal;
  private TrapezoidProfile.State m_elbowSetpoint;
  private PIDController m_elbowPid = new PIDController(kP, kI, kD);
  private double m_elbowVoltage;

  // Wrist
  private TimedServo m_servoWrist =
      new TimedServo(
          Constants.TrapConstants.kServoWristId, 260, Constants.TrapConstants.kWristSetZero);

  // Finger
  private TimedServo m_servoFinger =
      new TimedServo(
          Constants.TrapConstants.kServoFingerId, 260, Constants.TrapConstants.kFingerOpened);

  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kLimitSwitchId);

  public boolean setZero = false;

  public Trap() {
    m_shoulder.restoreFactoryDefaults();
    m_shoulderSetpoint = new TrapezoidProfile.State(m_shoulderEncoder.getPosition(), 0);
    m_shoulderGoal = new TrapezoidProfile.State(m_shoulderEncoder.getPosition(), 0);

    m_elbow.restoreFactoryDefaults();
    m_elbowSetpoint = new TrapezoidProfile.State(m_elbowEncoder.getPosition(), 0);
    m_elbowGoal = new TrapezoidProfile.State(m_elbowEncoder.getPosition(), 0);

    m_shoulderPid.setIZone(20.0 / 360.0);

    m_shoulder.setInverted(true);
    m_elbow.setInverted(true);

    DataLogManager.start();

    // Set up custom log entries
    DataLog log = DataLogManager.getLog();

    m_shoulderThetaCurrentLog = new DoubleLogEntry(log, "/Trap/CurrentShoulderTheta");
    m_shoulderThetaComputedLog = new DoubleLogEntry(log, "/Trap/ComputedShoulderTheta");

    m_shoulderOmegaCurrentLog = new DoubleLogEntry(log, "/Trap/CurrentShoulderOmega");
    m_shoulderOmegaComputedLog = new DoubleLogEntry(log, "/Trap/ComputedShoulderOmega");

    m_shoulderVoltageLog = new DoubleLogEntry(log, "/Trap/ShoulderVoltage");

    m_elbowThetaCurrentLog = new DoubleLogEntry(log, "/Trap/CurrentElbowTheta");
    m_elbowThetaComputedLog = new DoubleLogEntry(log, "/Trap/ComputedElbowTheta");

    m_elbowOmegaCurrentLog = new DoubleLogEntry(log, "/Trap/CurrentElbowOmega");
    m_elbowOmegaComputedLog = new DoubleLogEntry(log, "/Trap/ComputedElbowOmega");

    m_elbowVoltageLog = new DoubleLogEntry(log, "/Trap/ElbowVoltage");

    m_servoWrist.setZero();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setZero = false;
    } else {

      m_shoulderThetaCurrentLog.append((m_shoulderEncoder.getPosition() - m_zeroPosShoulder) * 360);
      m_shoulderOmegaCurrentLog.append(m_shoulderEncoder.getVelocity() * 360);

      m_shoulderSetpoint = m_shoulderProfile.calculate(kDt, m_shoulderSetpoint, m_shoulderGoal);
      m_shoulderThetaComputedLog.append((m_shoulderSetpoint.position - m_zeroPosShoulder) * 360);
      m_shoulderOmegaComputedLog.append(m_shoulderSetpoint.velocity * 360);

      m_shoulderVoltage =
          m_shoulderPid.calculate(m_shoulderEncoder.getPosition(), m_shoulderSetpoint.position);

      //      m_shoulderVoltage += Math.signum(m_shoulderPid.getPositionError()) * 0.5;

      m_shoulderVoltage +=
          m_shoulderFF.calculate(
              (m_shoulderSetpoint.position - m_zeroPosShoulder) * 360,
              m_shoulderSetpoint.velocity * 360);
      m_shoulderVoltage = MathUtil.clamp(m_shoulderVoltage, -10, 10);
      m_shoulderVoltageLog.append(m_shoulderVoltage);
      m_shoulder.setVoltage(m_shoulderVoltage);

      // elbow joint

      m_elbowThetaCurrentLog.append((m_elbowEncoder.getPosition() - m_zeroPosElbow) * 360);
      m_elbowOmegaCurrentLog.append(m_elbowEncoder.getVelocity() * 360);

      m_elbowSetpoint = m_elbowProfile.calculate(kDt, m_elbowSetpoint, m_elbowGoal);
      m_elbowThetaComputedLog.append((m_elbowSetpoint.position - m_zeroPosElbow) * 360);
      m_elbowOmegaComputedLog.append(m_elbowSetpoint.velocity * 360);

      m_elbowVoltage = m_elbowPid.calculate(m_elbowEncoder.getPosition(), m_elbowSetpoint.position);
      m_elbowVoltage = MathUtil.clamp(m_elbowVoltage, -10, 10);
      m_elbowVoltageLog.append(m_elbowVoltage);
      m_elbow.setVoltage(m_elbowVoltage);
    }
  }

  public Boolean shoulderReachedPos() {
    return Math.abs(m_shoulderEncoder.getPosition() - m_shoulderGoal.position) > 0.05;
  }

  public Boolean elbowReachedPos() {
    return Math.abs(m_elbowEncoder.getPosition() - m_elbowGoal.position) > 0.05;
  }

  public Command elbowIncrease() {
    return this.elbowMoveRel(0.02);
  }

  public Command elbowDecrease() {
    return this.elbowMoveRel(-0.02);
  }

  public Command shoulderIncrease() {
    return this.shoulderMoveRel(0.02);
  }

  public Command shoulderDecrease() {
    return this.shoulderMoveRel(-0.02);
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the SHOULDER motor
   */
  public Command shoulderMoveRel(double delta) {
    return this.runOnce(
        () -> {
          var target = m_shoulderGoal.position + delta;
          if (target < kshoulderDeadZoneBegin) m_shoulderGoal.position = kshoulderDeadZoneBegin;
          else if (target > kshoulderDeadZoneEnd) m_shoulderGoal.position = kshoulderDeadZoneEnd;
          else m_shoulderGoal.position = target;
        });
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the ELBOW motor
   */
  public Command elbowMoveRel(double delta) {
    return this.runOnce(
        () -> {
          var target = m_elbowGoal.position + delta;
          if (target < kelbowDeadZoneBegin) m_elbowGoal.position = kelbowDeadZoneBegin;
          else if (target > kelbowDeadZoneEnd) m_elbowGoal.position = kelbowDeadZoneEnd;
          else m_elbowGoal.position = target;
        });
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the SHOULDER motor
   */
  public Command shoulderMoveTo(double absolute) {
    return this.runOnce(
        () -> {
          if (absolute < kshoulderDeadZoneBegin) m_shoulderGoal.position = kshoulderDeadZoneBegin;
          else if (absolute > kshoulderDeadZoneEnd) m_shoulderGoal.position = kshoulderDeadZoneEnd;
          else m_shoulderGoal.position = absolute;
        });
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the ELBOW motor
   */
  public Command elbowMoveTo(double absolute) {
    return this.runOnce(
        () -> {
          if (absolute < kelbowDeadZoneBegin) m_elbowGoal.position = kelbowDeadZoneBegin;
          else if (absolute > kelbowDeadZoneEnd) m_elbowGoal.position = kelbowDeadZoneEnd;
          else m_elbowGoal.position = absolute;
        });
  }

  // position throughout game

  public Command setZero() {
    return Commands.sequence(
        elbowMoveTo(Constants.TrapConstants.kElbowSetZero),
        shoulderMoveTo(Constants.TrapConstants.kShoulderSetZero));
  }

  public Command setZeroGrab() {
    return Commands.sequence(
        elbowMoveTo(Constants.TrapConstants.kElbowSetZeroGrabSmall),
        elbowMoveTo(Constants.TrapConstants.kElbowSetZeroGrab),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristSetZeroGrabSmall)),
        new WaitCommand(0.5),
        shoulderMoveTo(Constants.TrapConstants.kShoulderSetZeroGrab),
        new WaitCommand(2.3),
        this.runOnce(() -> m_servoWrist.setZero()),
        new WaitCommand(m_servoWrist.travelTime()),
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened)),
        this.runOnce(() -> setZero = true));
  }

  public Command grabPosition() {
    return Commands.sequence(
        elbowMoveTo(Constants.TrapConstants.kElbowGrabPosition),
        new WaitCommand(0.5),
        shoulderMoveTo(Constants.TrapConstants.kShoulderGrabPosition),
        new WaitCommand(1.0),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristGrabPosition)));
  }

  public Command storeNote() {
    return Commands.sequence(
        elbowMoveTo(Constants.TrapConstants.kElbowStoreNoteSmall),
        new WaitCommand(0.4),
        shoulderMoveTo(Constants.TrapConstants.kShoulderStoreNoteSmall),
        new WaitCommand(0.4),
        elbowMoveTo(Constants.TrapConstants.kElbowStoreNote),
        shoulderMoveTo(Constants.TrapConstants.kShoulderStoreNote),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristStoreNote)));
  }

  // command to close the finger in the sequence trapshoot if deleted finger will never close and
  // the note won't stay in.
  public Command closeFinger() {
    return this.runOnce(() -> new WaitUntilCommand(() -> !m_limitSwitch.get()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed));
  }

  // WARNING: this may need some delayed positions be carefull when testing to not destroy
  // everything :D
  public Command prepareToClimb() {
    return Commands.sequence(
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed)),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb1),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb1),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb2),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb2),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb3),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb3),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb4),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb4),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb5),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb5),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb6),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristPrepareToClimb)));
  }

  // // position to dunk the note in the trap
  public Command dunkNote() {
    return Commands.sequence(
        shoulderMoveTo(Constants.TrapConstants.kShoulderDunkNote),
        elbowMoveTo(Constants.TrapConstants.kElbowDunkNote),
        new WaitCommand(1),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristDunkNote)),
        new WaitCommand(2),
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened)));
  }

  public Command hitNote() {
    return Commands.sequence(
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed)),
        shoulderMoveTo(Constants.TrapConstants.kShoulderHitNote),
        elbowMoveTo(Constants.TrapConstants.kElbowHitNote),
        new WaitCommand(1),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristHitNote)));
  }

  // // position arm is in after dunking the note so it is ready to disable and doesn't hit
  // anywhere.
  // // needs to be before the disable2 because it lifts the arm up before it folds the wrist and
  // // lowers the elbow in disable2
  public Command prepareToDisable() {
    return Commands.sequence(
        shoulderMoveTo(Constants.TrapConstants.kShoulderDisable),
        elbowMoveTo(Constants.TrapConstants.kElbowDisable),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristDisable)));
  }

  // // manual control to fine tune arm positions using pov on joystick.
  // public Command manualControl(Joint j, boolean increase) {
  //   return this.runOnce(
  //       () -> {
  //         var new_angle = m_jointArray[j.ordinal()].getAngle();
  //         if (increase) new_angle = new_angle < 180 ? new_angle + 1 : 180;
  //         else new_angle = new_angle > 0 ? new_angle - 1 : 0;
  //         var lambda_angle = new_angle; // making it effectively final so java lambda is happy..
  //         // :)
  //         m_jointArray[j.ordinal()].setAngle(lambda_angle);
  //       });
  // }

  // used in trap shoot sequence to detect note in the arm
  public boolean trapHasNote() {
    return !m_limitSwitch.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Shoulder pos", () -> m_shoulderEncoder.getPosition(), null);

    builder.addDoubleProperty("Shoulder target", () -> m_shoulderSetpoint.position, null);

    builder.addDoubleProperty("Elbow pos", () -> m_elbowEncoder.getPosition(), null);
    builder.addDoubleProperty("Elbow target", () -> m_elbowSetpoint.position, null);
  }
}
