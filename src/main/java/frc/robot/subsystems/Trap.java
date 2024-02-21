package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  public boolean setZero = false;

  public enum Joint {
    SHOULDER, // Idle orange, default
    ELBOW, // Intake white slow flash, when intake is rolling
    WRIST, // Detected note green, note in beam cutter triggered
    FINGER, // Aim activated white quick flash, with vision aim function running
  }

  private boolean m_debug = false;

  private final TimedServo m_jointArray[] =
      new TimedServo[] {m_servoShoulder, m_servoElbow, m_servoWrist, m_servoFinger};

  /* TODO: WHY???
  public void initDefaultCommand() {

    setDefaultCommand(grabPosition());
  }
  */

  // @Override
  public void periodic() {
    if (m_debug) {
      for (Joint j : Joint.values()) {
        var s = m_jointArray[j.ordinal()];
        SmartDashboard.putString(
            String.format("%s : %d", j.name(), s.getChannel()),
            String.format("@ :%f deg", s.getAngle()));
      }
    }

    if (DriverStation.isDisabled()) {
      setZero = false;
    }
  }

  public Command setZero() {
    return this.runOnce(() -> m_servoWrist.setZero())
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoElbow.setZero())
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShouldersetZeroDelayed))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> m_servoShoulder.setZero())
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
        .andThen(new WaitCommand(m_servoFinger.travelTime()).andThen(() -> setZero = true));
  }

  public Command storeNote() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderstoreNote))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowstoreNote))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWriststoreNote))
        .andThen(new WaitCommand(m_servoWrist.travelTime()))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed))
        .andThen(new PrintCommand("limit switch on"));
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

  public Command prepareToClimb() {
    return this.runOnce(
            () -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimbdelayed1))
        .andThen(new WaitCommand(0.2))
        .andThen(
            () ->
                m_servoShoulder.setAngle(
                    Constants.TrapConstants.kangleShoulderprepareToClimbdelayed1))
        .andThen(new WaitCommand(0.2))
        .andThen(
            () -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimbdelayed2))
        .andThen(new WaitCommand(0.2))
        .andThen(
            () ->
                m_servoShoulder.setAngle(
                    Constants.TrapConstants.kangleShoulderprepareToClimbdelayed2))
        .andThen(new WaitCommand(0.2))
        .andThen(
            () -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimbdelayed3))
        .andThen(new WaitCommand(0.2))
        .andThen(
            () ->
                m_servoShoulder.setAngle(
                    Constants.TrapConstants.kangleShoulderprepareToClimbdelayed3))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowprepareToClimb))
        .andThen(new WaitCommand(0.2))
        .andThen(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderprepareToClimb))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerClosed))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristprepareToClimb))
        .andThen(new WaitCommand(m_servoWrist.travelTime()));
  }

  public Command dunkNote() {
    return this.runOnce(
            () -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderdunkNote))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowdunkNote))
        .andThen(new WaitCommand(1))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristdunkNote))
        .andThen(new WaitCommand(0.6))
        .andThen(() -> m_servoFinger.setAngle(Constants.TrapConstants.kfingerOpened))
        .andThen(new WaitCommand(0.5))
        .andThen(() -> m_servoShoulder.setAngle(Constants.TrapConstants.kangleShoulderdisable))
        .andThen(new WaitCommand(m_servoShoulder.travelTime()))
        .andThen(() -> m_servoElbow.setAngle(Constants.TrapConstants.kangleElbowdisable))
        .andThen(new WaitCommand(m_servoElbow.travelTime()))
        .andThen(() -> m_servoWrist.setAngle(Constants.TrapConstants.kangleWristdisable))
        .andThen(new WaitCommand(0.6));
  }

  public Command manualControl(Joint j, boolean increase) {
    return this.runOnce(
        () -> {
          var new_angle = m_jointArray[j.ordinal()].getAngle();
          if (increase) new_angle = new_angle < 180 ? new_angle + 1 : 180;
          else new_angle = new_angle > 0 ? new_angle - 1 : 0;
          var lambda_angle = new_angle; // making it effectively final so java lambda is happy..
          // :)
          m_jointArray[j.ordinal()].setAngle(lambda_angle);
        });
  }

  public boolean trapHasNote() {
    return !m_limitSwitch.get();
  }
}

// 250ms for shoulder

// uwu
// :3
