package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

  private CANSparkMax m_shoulder =
      new CANSparkMax(Constants.TrapConstants.kShoulderId, MotorType.kBrushed);
  private CANSparkMax m_elbow =
      new CANSparkMax(Constants.TrapConstants.kElbowId, MotorType.kBrushed);
  private RelativeEncoder m_shoulderEncoder = m_shoulder.getEncoder();
  private RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();

  private TimedServo m_servoWrist =
      new TimedServo(
          Constants.TrapConstants.kServoWristId, 260, Constants.TrapConstants.kWristSetZero);
  private TimedServo m_servoFinger =
      new TimedServo(
          Constants.TrapConstants.kServoFingerId, 260, Constants.TrapConstants.kFingerOpened);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kLimitSwitchId);

  public boolean setZero = false;

  private double m_shoulderSpeed = 0.0;
  private double m_elbowSpeed = 0.0;
  private double m_desiredSpeed = 0.3;

  public Trap() {
    m_shoulder.restoreFactoryDefaults();
    m_elbow.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    System.out.println(m_shoulderEncoder.getPosition());
    System.out.println(m_elbowEncoder.getPosition());

    if (DriverStation.isDisabled()) {
      setZero = false;
    }

    m_elbow.set(m_elbowSpeed);
    m_shoulder.set(m_shoulderSpeed);
  }

  /**
   * @param target: the desired position in positive numbers
   * @param speed: the desired speed between 0 and 1 (the code converts the velocity in the first
   *     runOnce())
   * @return the movement of the motor to desired position with desired speed for the SHOULDER motor
   */
  public Command shoulderMoveTo(double target, double speed) {
    return Commands.sequence(
        this.runOnce(
            () -> {
              if (target > m_shoulderEncoder.getPosition()) {
                m_shoulderSpeed = speed;
              } else if (target < m_shoulderEncoder.getPosition()) {
                m_shoulderSpeed -= speed;
              } else m_shoulderSpeed = 0;
            }),
        new WaitUntilCommand(() -> Math.abs(m_shoulderEncoder.getPosition() - target) <= 0.015),
        this.runOnce(() -> m_shoulderSpeed = 0));
  }

  /**
   * @param target: the desired position in positive numbers
   * @param speed: the desired speed between 0 and 1 (the code converts the velocity in the first
   *     runOnce())
   * @return the movement of the motor to desired position with desired speed for the ELBOW motor
   */
  public Command elbowMoveTo(double target, double speed) {
    return Commands.sequence(
        this.runOnce(
            () -> {
              if (target > m_elbowEncoder.getPosition()) {
                m_elbowSpeed = speed;
              } else if (target < m_elbowEncoder.getPosition()) {
                m_elbowSpeed -= speed;
              } else m_elbowSpeed = 0;
            }),
        new WaitUntilCommand(() -> Math.abs(m_elbowEncoder.getPosition() - target) <= 0.015),
        this.runOnce(() -> m_elbowSpeed = 0));
  }

  // position throughout game
  // TODO this has to be modified accordingly to the new motors mostly with time stuff
  public Command setZero() {
    return this.runOnce(() -> shoulderMoveTo(0.0, m_desiredSpeed))
        .andThen(() -> elbowMoveTo(0.0, m_desiredSpeed))
        .andThen(() -> m_servoWrist.setZero())
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerOpened))
        .andThen(new WaitCommand(m_servoFinger.travelTime()).andThen(() -> setZero = true));
  }

  // TODO this also has to be modified
  // position to store the note in the robot so robot can still pass under chain
  public Command storeNote() {
    return this.runOnce(
            () -> shoulderMoveTo(Constants.TrapConstants.kShoulderStoreNote, m_desiredSpeed))
        .andThen(() -> elbowMoveTo(Constants.TrapConstants.kElbowStoreNote, m_desiredSpeed))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kWristStoreNote))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kFingerClosed));
  }

  // // position to grab note from intake.
  // public Command grabPosition() {
  //   return this.runOnce(
  //           () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldergrabPosition))
  //       .andThen(new WaitCommand(0.1))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition))
  //       .andThen(new WaitCommand(0.3))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristgrabPosition))
  //       .andThen(new WaitCommand(m_servoWrist.travelTime()))
  //       .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened));
  // }

  // public Command pushNote() {
  //   return this.runOnce(
  //           () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderpushNote))
  //       .andThen(new WaitCommand(0.1))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowpushNote))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristpushNote));
  // }

  // // weird command don't delete because it works in trap shoot sequence. it doesn't pick up on
  // limit
  // // switch for some reason but it is placed after the has note in the sequence. it is basically
  // // just a command to close the finger.
  // public Command closeFinger() {
  //   return this.runOnce(() -> new WaitUntilCommand(() -> !m_limitSwitch.get()))
  //       .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed));
  // }

  // // position to lift arm up BEFORE elevator so it doesn't hit leds (with a lot of delayed things
  // so
  // // it doesn't hit too much (to be fine tuned before competition so it doesn't hit anywhere))
  // public Command prepareToClimb() {
  //   return this.runOnce(
  //           () ->
  // m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimbdelayed1))
  //       .andThen(
  //           () ->
  //               m_servoShoulder.setAngle(
  //                   Constants.TrapConstants.kangleShoulderprepareToClimbdelayed1))
  //       .andThen(new WaitCommand(0.1))
  //       .andThen(
  //           () ->
  // m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimbdelayed2))
  //       .andThen(
  //           () ->
  //               m_servoShoulder.setAngle(
  //                   Constants.TrapConstants.kangleShoulderprepareToClimbdelayed2))
  //       .andThen(new WaitCommand(0.1))
  //       .andThen(
  //           () ->
  // m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimbdelayed3))
  //       .andThen(
  //           () ->
  //               m_servoShoulder.setAngle(
  //                   Constants.TrapConstants.kangleShoulderprepareToClimbdelayed3))
  //       .andThen(new WaitCommand(0.1))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimb))
  //       .andThen(
  //           () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderprepareToClimb))
  //       .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristprepareToClimb));
  // }

  // // position to dunk the note in the trap
  // public Command dunkNote() {
  //   return this.runOnce(
  //           () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderdunkNote))
  //       .andThen(new WaitCommand(m_servoShoulder.travelTime()))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowdunkNote))
  //       .andThen(new WaitCommand(0.32))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristdunkNote))
  //       .andThen(new WaitCommand(0.5))
  //       // Note should now be stuck in the trap at an angle
  //       // Trying to push it through the trap. Moving the wrist up a bit and the elbow down
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleWristFinalPush))
  //       .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowFinalPush))
  //       .andThen(new WaitCommand(0.2))
  //       // Opening up the finger
  //       .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
  //       .andThen(new WaitCommand(0.1))
  //       .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristprepareToClimb))
  //       .andThen(new WaitCommand(0.1));
  // }

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
}
