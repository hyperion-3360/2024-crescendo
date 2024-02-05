package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private Servo m_servoShoulder = new Servo(Constants.TrapConstants.kservoShoulderId);
  private Servo m_servoElbow = new Servo(Constants.TrapConstants.kservoElbowId);
  private Servo m_servoWrist = new Servo(Constants.TrapConstants.kservoWristId);
  private Servo m_servoFinger = new Servo(Constants.TrapConstants.kservoFingerId);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kfingerlimitswitchId);

  public Trap() {
    setZero();
  }

  public void setZero() {

    m_servoShoulder.setAngle(
        Constants.TrapConstants.kangleShouldersetZero); // arm position during the game
    m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowsetZero);
    m_servoWrist.setAngle(Constants.TrapConstants.kangleWristsetZero);
  }

  public void grabPosition() {

    m_servoShoulder.setAngle(
        Constants.TrapConstants.kangleShouldergrabPosition); // arm position when grabing note
    m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition);
    m_servoWrist.setAngle(Constants.TrapConstants.kangleElbowgrabPosition);
  }

  public void scoreNote() {

    m_servoShoulder.setAngle(
        Constants.TrapConstants.kangleShoulderscoreNote); // arm position when grabing note
    m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowscoreNote);
    m_servoWrist.setAngle(Constants.TrapConstants.kangleWristscoreNote);
    m_servoFinger.setAngle(Constants.TrapConstants.kangleFingerscoreNote);
  }

  @Override
  public void periodic() {
    //  System.out.println(m_servoShoulder.get());

    if (!m_limitSwitch.get()) m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed);
    else m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened);
  }

  public Command grabNote() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldergrabPosition))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleElbowgrabPosition));
  }
}
