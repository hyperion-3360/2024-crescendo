package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        .andThen(() -> m_servoElbow.setZero())
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldersetZeroDelayed))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()));
        .andThen(() -> m_servoShoulder.setZero())
        .andThen(new WaitCommand(m_servoShoulder.travelTime()));
  }

  public Command grabPosition() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldergrabPosition))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristgrabPosition))
        .andThen(new WaitCommand(m_servoWrist.travelTime()));
  }

  public Command scoreNote() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderscoreNote))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowscoreNote))
        .andThen(new WaitCommand(m_servoElbow.travelTime(0.5)))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristscoreNote))
        .andThen(new WaitCommand(m_servoWrist.travelTime(0.5)))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
        .andThen(new WaitCommand(m_servoFinger.travelTime(0.5)));
  }

  public Command limitSwitch() {
    if (!m_limitSwitch.get())
      return this.runOnce(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened));
    return this.runOnce(() -> m_limitSwitch.get());
  }

  public Command test0() {
    return this.runOnce(
        () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldersetZero));
  }

  public Command test50() {
    return this.runOnce(
        () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldergrabPosition));
  }
}

// 250ms for shoulder

// uwu
