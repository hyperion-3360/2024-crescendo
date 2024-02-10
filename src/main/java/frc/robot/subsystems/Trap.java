package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TimedServo;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private TimedServo m_servoShoulder =
      new TimedServo(Constants.TrapConstants.kservoShoulderId, 0.0);
  private TimedServo m_servoElbow =
      new TimedServo(
          Constants.TrapConstants.kservoElbowId, 0.0, Constants.TrapConstants.kangleElbowsetZero);
  private TimedServo m_servoWrist =
      new TimedServo(
          Constants.TrapConstants.kservoWristId, 0.0, Constants.TrapConstants.kangleWristsetZero);
  private TimedServo m_servoFinger =
      new TimedServo(
          Constants.TrapConstants.kservoFingerId, 0.0, Constants.TrapConstants.kfingerOpened);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kfingerlimitswitchId);

  public void initDefaultCommand() {

    setDefaultCommand(grabPosition());
  }

  @Override
  public void periodic() {}

  public Command setZero() {
    return this.run(() -> m_servoWrist.setZero())
        .until(() -> m_servoWrist.isDone(0.5))
        .andThen(() -> m_servoElbow.setZero())
        .until(() -> m_servoElbow.isDone(0.5))
        .andThen(() -> m_servoShoulder.setZero())
        .until(() -> m_servoShoulder.isDone());
  }

  public Command grabPosition() {
    return this.run(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldergrabPosition))
        .until(() -> m_servoShoulder.isDone())
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition))
        .until(() -> m_servoElbow.isDone(0.5))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristgrabPosition))
        .until(() -> m_servoWrist.isDone(0.5));
  }

  public Command scoreNote() {
    return this.run(() -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderscoreNote))
        .until(() -> m_servoShoulder.isDone())
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowscoreNote))
        .until(() -> m_servoElbow.isDone(0.5))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristscoreNote))
        .until(() -> m_servoWrist.isDone(0.5))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
        .until(() -> m_servoFinger.isDone(0.5));
  }

  public Command limitSwitch() {
    if (!m_limitSwitch.get())
      return this.run(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened));
    return this.run(() -> m_limitSwitch.get());
  }
}
// uwu
