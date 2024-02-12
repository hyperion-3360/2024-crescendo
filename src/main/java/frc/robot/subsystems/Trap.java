package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.TimedServo;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private TimedServo m_servoShoulder =
      new TimedServo(Constants.TrapConstants.kservoShoulderId, 260);
  private TimedServo m_servoElbow =
      new TimedServo(
          Constants.TrapConstants.kservoElbowId, 260, Constants.TrapConstants.kangleElbowsetZero);
  private TimedServo m_servoWrist =
      new TimedServo(
          Constants.TrapConstants.kservoWristId, 260, Constants.TrapConstants.kangleWristsetZero);
  private TimedServo m_servoFinger =
      new TimedServo(
          Constants.TrapConstants.kservoFingerId, 260, Constants.TrapConstants.kfingerOpened);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kfingerlimitswitchId);

  public void initDefaultCommand() {

    setDefaultCommand(grabPosition());
  }

  @Override
  public void periodic() {}

  public Command setZero() {
    return this.runOnce(() -> m_servoWrist.setZero())
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldersetZeroDelayed))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> m_servoElbow.setZero())
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoShoulder.setZero())
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
        .andThen(new WaitCommand(m_servoFinger.travelTime()));
  }

  public Command grabPosition() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldergrabPosition))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristgrabPosition))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(new WaitUntilCommand(() -> !m_limitSwitch.get()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed));
  }

  public Command scoreNote() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderscoreNote))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowscoreNote))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristscoreNote))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
        .andThen(new WaitCommand(m_servoFinger.travelTime()));
  }

  public Command storeNote() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderstoreNote))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowstoreNote))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWriststoreNote))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(new WaitUntilCommand(() -> !m_limitSwitch.get()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed));
  }

  public Command prepareToClimb() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderprepareToClimb))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimb))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristprepareToClimb))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(new WaitUntilCommand(() -> !m_limitSwitch.get()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed));
  }

  public boolean trapHasNote() {
    return !m_limitSwitch.get();
  }
}

// 250ms for shoulder

// uwu
// :3
