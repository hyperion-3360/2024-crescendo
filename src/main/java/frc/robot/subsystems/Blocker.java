package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blocker extends SubsystemBase {

  private final double kIntakeHookAngleOpen = 0.0;
  private final double kIntakeHookAngleClose = 23;

  private Servo m_servoBlocker = new Servo(Constants.TrapConstants.kservoBlockerId);

  public Blocker() {
    m_servoBlocker.setAngle(kIntakeHookAngleClose);
  }

  @Override
  public void periodic() {}

  // si il a une note il attend 3 secondes avant de lever
  public Command hookIntake() {
    return this.runOnce(() -> m_servoBlocker.setAngle(kIntakeHookAngleClose));
  }

  // si il a une note il attend 3 secondes avant de lever
  public Command hookRelease() {
    return this.runOnce(() -> m_servoBlocker.setAngle(kIntakeHookAngleOpen));
  }
}