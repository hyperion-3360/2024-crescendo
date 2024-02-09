package frc.lib.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;

public class TimedServo extends Servo {

  private double m_angularSpeed = 0.2;
  private double m_completedTime = 0.0;
  private double m_currentAngle = 0.0;

  public TimedServo(int channel, double angularSpeed) {
    super(channel);
    this.m_angularSpeed = angularSpeed;
  }

  public TimedServo(int channel, double angularSpeed, double zero) {
    super(channel);
    this.m_angularSpeed = angularSpeed;
    this.m_currentAngle = zero;
  }

  @Override
  public void setAngle(double angle) {
    var delta = Math.abs(m_currentAngle - angle);

    m_completedTime = delta * m_angularSpeed + RobotController.getFPGATime();
    super.setAngle(angle);
    m_currentAngle = angle;
  }

  public boolean isDone() {
    return RobotController.getFPGATime() > m_completedTime;
  }
}
