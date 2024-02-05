package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private Servo m_servoShoulder = new Servo(Constants.TrapConstants.kservoShoulderId);
  private Servo m_servoElbow = new Servo(Constants.TrapConstants.kservoElbowId);
  private Servo m_servoWrist = new Servo(Constants.TrapConstants.kservoWristId);
  private Servo m_servoFinger = new Servo(Constants.TrapConstants.kservoFingerId);
  public DigitalInput m_limitSwitch =
      new DigitalInput(Constants.TrapConstants.kfingerlimitswitchId);

  public Trap() {}

  public Command setZero() {
    return this.runOnce(
        () -> {
          m_servoShoulder.setAngle(
              Constants.TrapConstants.kangleShouldersetZero); // arm position during the game
          m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowsetZero);
          m_servoWrist.setAngle(Constants.TrapConstants.kangleWristsetZero);
        });
  }

  public Command grabPosition() {
    return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              m_servoShoulder.setAngle(
                  Constants.TrapConstants
                      .kangleShouldergrabPosition); // arm position when grabing note
              m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition);
              m_servoWrist.setAngle(Constants.TrapConstants.kangleWristgrabPosition);
            }));
  }

  public Command scoreNote() {
    return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              m_servoShoulder.setAngle(
                  Constants.TrapConstants
                      .kangleShoulderscoreNote); // arm position when grabing note
              m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowscoreNote);
              m_servoWrist.setAngle(Constants.TrapConstants.kangleWristscoreNote);
              m_servoFinger.setAngle(Constants.TrapConstants.kangleFingerscoreNote);
            }));
  }

  @Override
  public void periodic() {

    this.setZero();

    if (!m_limitSwitch.get()) m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed);
    else m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened);
  }
}
