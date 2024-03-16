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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.TimedServo;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private final double kshoulderDeadZoneBegin = 0.1;
  private final double kshoulderDeadZoneEnd = 0.42;
  private final double kelbowDeadZoneBegin = 0.05;
  private final double kelbowDeadZoneEnd = 0.50;
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

  private final double skP = 70;
  private final double skI = 30;
  private final double skD = 0;

  private final double ekP = 40;
  private final double ekI = 30;
  private final double ekD = 2;

  private PIDController m_shoulderPid = new PIDController(skP, skI, skD);
  private PIDController m_elbowPid = new PIDController(ekP, ekI, ekD);

  // both shoulder and elbow are assumed to have their absolute 0 in the mechanical dead zone
  // this deadzone is expected to be at least 0.2 rotation wide (from 0.9 to 0.1)
  private double m_shoulderPos = m_shoulderEncoder.getPosition();
  private double m_elbowPos = m_elbowEncoder.getPosition();

  public Trap() {
    m_shoulder.restoreFactoryDefaults();
    m_elbow.restoreFactoryDefaults();

    m_shoulder.setInverted(true);
    m_elbow.setInverted(true);
  }

  @Override
  public void periodic() {
    System.out.println(
        "elbow " + m_elbowEncoder.getPosition() + " shoulder " + m_shoulderEncoder.getPosition());
    if (DriverStation.isDisabled()) {
      setZero = false;
    } else {

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
  }

  public Command elbowIncrease() {
    return this.elbowMoveRel(0.05);
  }

  public Command elbowDecrease() {
    return this.elbowMoveRel(-0.05);
  }

  public Command shoulderIncrease() {
    return this.shoulderMoveRel(0.05);
  }

  public Command shoulderDecrease() {
    return this.shoulderMoveRel(-0.05);
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the SHOULDER motor
   */
  public Command shoulderMoveRel(double delta) {
    return this.runOnce(
        () -> {
          var target = m_shoulderPos + delta;
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
  public Command elbowMoveRel(double delta) {
    return this.runOnce(
        () -> {
          var target = m_elbowPos + delta;
          m_elbowPos =
              ((target < kelbowDeadZoneBegin) || (target > kelbowDeadZoneEnd))
                  ? m_elbowPos
                  : target;
        });
  }

  /**
   * @param target: the desired position in positive numbers
   * @return the movement of the motor to desired position with desired speed for the SHOULDER motor
   */
  public Command shoulderMoveTo(double absolute) {
    return this.runOnce(
        () -> {
          var target = absolute;
          System.out.println("shoulder " + target);
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
  public Command elbowMoveTo(double absolute) {
    return this.runOnce(
        () -> {
          var target = absolute;
          System.out.println("elbow " + target);
          m_elbowPos =
              ((target < kelbowDeadZoneBegin) || (target > kelbowDeadZoneEnd))
                  ? m_elbowPos
                  : target;
        });
  }

  // position throughout game

  public Command setZero() {
    return Commands.sequence(
        shoulderMoveTo(Constants.TrapConstants.kShoulderSetZero),
        elbowMoveTo(Constants.TrapConstants.kElbowSetZero),
        this.runOnce(() -> elbowMoveTo(Constants.TrapConstants.kElbowSetZero)),
        this.runOnce(() -> m_servoWrist.setZero()),
        new WaitCommand(m_servoWrist.travelTime()),
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened)),
        new WaitCommand(m_servoFinger.travelTime()),
        this.runOnce(() -> setZero = true));
  }

  public Command grabPosition() {
    return Commands.sequence(
        shoulderMoveTo(Constants.TrapConstants.kShoulderGrabPosition),
        elbowMoveTo(Constants.TrapConstants.kElbowGrabPosition),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristGrabPosition)));
  }

  public Command storeNote() {
    return Commands.sequence(
        shoulderMoveTo(Constants.TrapConstants.kShoulderStoreNote),
        elbowMoveTo(Constants.TrapConstants.kElbowStoreNote),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kShoulderStoreNote)));
  }

  // TODO this also has to be modified
  // position to store the note in the robot so robot can still pass under chain
  // public Command storeNote() {
  //   return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderStoreNote))
  //       .andThen(() -> elbowMoveTo(Constants.TrapConstants.kElbowStoreNote))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristStoreNote))
  //       .andThen(new WaitCommand(m_servoWrist.travelTime()))
  //       .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed));
  // }

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

  // // position to grab note from intake.
  // public Command grabPosition() {
  //   return this.runOnce(() -> shoulderMoveTo(Constants.TrapConstants.kShoulderGrabPosition))
  //       .andThen(() -> shoulderMoveTo(Constants.TrapConstants.kElbowGrabPosition))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristGrabPosition))
  //       .andThen(new WaitCommand(m_servoWrist.travelTime()))
  //       .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened));
  // }

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
    return Commands.sequence(
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed)),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb1),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb1),
        new WaitCommand(.2),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb2),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb2),
        new WaitCommand(.2),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb3),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb3),
        new WaitCommand(.2),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb4),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb4),
        new WaitCommand(.2),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb5),
        elbowMoveTo(Constants.TrapConstants.kElbowPrepareToClimb5),
        new WaitCommand(.2),
        shoulderMoveTo(Constants.TrapConstants.kShoulderPrepareToClimb6),
        new WaitCommand(.2),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristPrepareToClimb)));
  }

  // // position to dunk the note in the trap
  public Command dunkNote() {
    return Commands.sequence(
        shoulderMoveTo(Constants.TrapConstants.kShoulderDunkNote),
        elbowMoveTo(Constants.TrapConstants.kElbowDunkNote),
        this.runOnce(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristDunkNote)),
        this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened)));
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
    builder.addDoubleProperty("Shoulder pos", () -> m_shoulderEncoder.getPosition(), null);
    builder.addDoubleProperty("Elbow pos", () -> m_elbowEncoder.getPosition(), null);
    builder.addDoubleProperty("Shoulder target", () -> m_shoulderPos, null);
    builder.addDoubleProperty("Elbow target", () -> m_elbowPos, null);
  }
}
