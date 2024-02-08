package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private Servo m_servoShoulder = new Servo(Constants.TrapConstants.kservoShoulderId);
  private Servo m_servoElbow = new Servo(Constants.TrapConstants.kservoElbowId);
  private Servo m_servoWrist = new Servo(Constants.TrapConstants.kservoWristId);
  private Servo m_servoFinger = new Servo(Constants.TrapConstants.kservoFingerId);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.TrapConstants.kfingerlimitswitchId);

  // private var m_timedServo = new TimedServo(TimedServo.TimedServo.);

  public Trap() {}

  @Override
  public void periodic() {

    // if (!m_limitSwitch.get() && isDone);
    // else m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened);
  }

  enum ServoZeroSeqStates {
    START,
    SHOULDER,
    ZERO
  }

  private ServoZeroSeqStates servoZeroState = ServoZeroSeqStates.START;
  private Double servoTimer = null;

  public Command setZero() {
    return this.run(
            () -> {
              if (servoZeroState == ServoZeroSeqStates.START) {

                servoTimer = Timer.getFPGATimestamp();
                m_servoShoulder.setAngle(
                    Constants.TrapConstants
                        .kangleShouldersetZeroDelayed); // delays the shoulder angle (goes to half
                // angle then total angle)
                servoZeroState = ServoZeroSeqStates.SHOULDER;

              } else if (servoZeroState == ServoZeroSeqStates.SHOULDER) {
                if (Timer.getFPGATimestamp() - servoTimer
                    > 0.3) { // 0.3 seconds or 300 milliseconds (it's not calulated but works)

                  servoZeroState = ServoZeroSeqStates.ZERO;
                  m_servoElbow.setAngle(
                      Constants.TrapConstants.kangleElbowsetZero); // all the complete angles
                  m_servoWrist.setAngle(Constants.TrapConstants.kangleWristsetZero);
                  m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldersetZero);
                }
              }
            })
        .until(
            () -> {
              return servoZeroState == ServoZeroSeqStates.ZERO;
            })
        .andThen(
            () -> {
              servoZeroState = ServoZeroSeqStates.START;
            });
  }

  public Command scoreNote() {
    return this.runOnce(
            () ->
                m_servoShoulder.setAngle(
                    Constants.TrapConstants.kangleShoulderscoreNote)) // arm position to score note
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowscoreNote))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristscoreNote));
  }

  public Command grabPosition() {
    return this.runOnce(
            () ->
                m_servoShoulder.setAngle(
                    Constants.TrapConstants
                        .kangleShouldergrabPosition)) // arm position grabing note (from shooter)
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowgrabPosition))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristgrabPosition))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kfingerOpened));
  }
}
