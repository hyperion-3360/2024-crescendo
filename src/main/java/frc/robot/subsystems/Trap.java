package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.TimedServo;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private final double kshoulderDeadZoneBegin = 0.1;
  private final double kshoulderDeadZoneEnd = 0.9;
  private final double kelbowDeadZoneBegin = 0.1;
  private final double kelbowDeadZoneEnd = 0.9;
  private final double kmotorAcceptablePosError = 0.01;

  private CANSparkMax m_shoulder =
      new CANSparkMax(Constants.TrapConstants.kShoulderId, MotorType.kBrushed);
  private CANSparkMax m_elbow =
      new CANSparkMax(Constants.TrapConstants.kElbowId, MotorType.kBrushed);
  private final SparkAbsoluteEncoder m_shoulderEncoder =
      m_shoulder.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkAbsoluteEncoder m_elbowEncoder = m_elbow.getAbsoluteEncoder(Type.kDutyCycle);

  private TimedServo m_servoWrist =
      new TimedServo(
          Constants.TrapConstants.kServoWristId, 260, Constants.TrapConstants.kWristSetZero);
  private TimedServo m_servoFinger =
      new TimedServo(
          Constants.TrapConstants.kServoFingerId, 260, Constants.TrapConstants.kFingerOpened);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kLimitSwitchId);

  public boolean setZero = false;

  private final double kP = 25;
  private final double kI = 5;
  private final double kD = 0;

  private PIDController m_shoulderPid = new PIDController(kP, kI, kD);
  private PIDController m_elbowPid = new PIDController(kP, kI, kD);

  // both shoulder and elbow are assumed to have their absolute 0 in the mechanical dead zone
  // this deadzone is expected to be at least 0.2 rotation wide (from 0.9 to 0.1)
  private double m_shoulderPos = kshoulderDeadZoneBegin;
  private double m_elbowPos = kelbowDeadZoneBegin;

  public Trap() {
    m_shoulderEncoder.setInverted(true);
    m_shoulder.setInverted(false);
    m_elbow.setInverted(false);
    m_elbowEncoder.setInverted(true);
    m_shoulder.restoreFactoryDefaults();
    m_elbow.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setZero = false;
    }

    var voltage =
        Math.abs(m_shoulderPos - m_shoulderEncoder.getPosition()) < kmotorAcceptablePosError
            ? 0.0
            : m_shoulderPid.calculate(m_shoulderEncoder.getPosition(), m_shoulderPos);

    m_shoulder.setVoltage(voltage);

    voltage =
        Math.abs(m_elbowPos - m_elbowEncoder.getPosition()) < kmotorAcceptablePosError
            ? 0.0
            : m_elbowPid.calculate(m_elbowEncoder.getPosition(), m_elbowPos);

    m_elbow.setVoltage(voltage);
  }

  public Command increase() {
    return this.shoulderMoveTo(m_shoulderEncoder.getPosition() + 0.1);
  }

  public Command decrease() {
    return this.shoulderMoveTo(m_shoulderEncoder.getPosition() - 0.1);
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the SHOULDER motor
   */
  public Command shoulderMoveTo(double target) {
    return this.runOnce(
        () -> {
          m_shoulderPos =
              ((target < kshoulderDeadZoneBegin) || (target > kshoulderDeadZoneEnd))
                  ? m_shoulderPos
                  : target;
        });
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the ELBOW motor
   */
  public Command elbowMoveTo(double target) {
    return this.runOnce(
        () -> {
          m_elbowPos =
              ((target < kelbowDeadZoneBegin) || (target > kelbowDeadZoneEnd))
                  ? m_elbowPos
                  : target;
        });
  }

  // position throughout game
  // TODO this has to be modified accordingly to the new motors mostly with time stuff
  public Command setZero() {
    return this.runOnce(() -> shoulderMoveTo(0.1))
        .andThen(() -> elbowMoveTo(0.1))
        .andThen(() -> m_servoWrist.setZero())
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened))
        .andThen(new WaitCommand(m_servoFinger.travelTime()).andThen(() -> setZero = true));
  }

  // TODO this also has to be modified
  // position to store the note in the robot so robot can still pass under chain
  public Command storeNote() {
    return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderStoreNote))
        .andThen(() -> elbowMoveTo(Constants.TrapConstants.kElbowStoreNote))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristStoreNote))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed));
  }

  // test commands to test each motor one by one
  public Command testShoulder() {
    return this.runOnce(() -> shoulderMoveTo(0.1));
  }

  public Command testElbow() {
    return this.runOnce(() -> elbowMoveTo(0.1));
  }

  public Command testWrist() {
    return this.runOnce(() -> m_servoWrist.setAngle(0.0));
  }

  public Command testFinger() {
    return this.runOnce(() -> m_servoFinger.setAngle(0.0));
  }

  // position to grab note from intake.
  public Command grabPosition() {
    return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderGrabPosition))
        .andThen(() -> shoulderMoveTo(Constants.TrapConstants.kElbowGrabPosition))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristGrabPosition))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened));
  }

  public Command pushNote() {
    return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderPushNote))
        .andThen(() -> elbowMoveTo(Constants.TrapConstants.kElbowPushNote))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristPushNote));
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
    return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb))
        .andThen(() -> elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristPrepareToClimb));
  }

  // // position to dunk the note in the trap
  public Command dunkNote() {
    return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderDunkNote))
        .andThen(() -> elbowMoveTo(Constants.TrapConstants.kElbowDunkNote))
        // Opening up the finger
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened))
        .andThen(new WaitCommand(0.1))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristDunkNote));
  }

  // // position arm is in after dunking the note so it is ready to disable and doesn't hit
  // anywhere.
  // // needs to be before the disable2 because it lifts the arm up before it folds the wrist and
  // // lowers the elbow in disable2
  // public Command prepareToDisable1() {
  //   return this.runOnce(
  //           () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderdisable1))
  //       .andThen(new WaitCommand(m_servoShoulder.travelTime()))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowdisable1))
  //       .andThen(new WaitCommand(m_servoElbow.travelTime()))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristdisable1));
  // }

  // // position that folds wrist and arm will rest on shooter.
  // public Command prepareToDisable2() {
  //   return this.runOnce(
  //           () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderdisable2))
  //       .andThen(new WaitCommand(m_servoShoulder.travelTime()))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowdisable2))
  //       .andThen(new WaitCommand(0.6))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristdisable2));
  // }

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
    builder.setSmartDashboardType("Big Arm");
    builder.addDoubleProperty("Shoulder pos", () -> m_shoulderPos, null);
    builder.addDoubleProperty("Elbow pos", () -> m_elbowPos, null);
  }
}
