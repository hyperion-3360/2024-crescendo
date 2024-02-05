package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blocker extends SubsystemBase {

  private Servo m_servoBlocker = new Servo(Constants.TrapConstants.kservoBlockerId);

  public Blocker() {}

  @Override
  public void periodic() {}

  // set l'angle a zero
  public Command setZero() {
    return this.runOnce(() -> m_servoBlocker.setAngle(70));
  }

  // si il a une note il attend 3 secondes avant de lever
  public Command letGoOfNote() {
    return this.runOnce(() -> m_servoBlocker.setAngle(130));
  }
}
