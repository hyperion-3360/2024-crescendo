package frc.lib.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class TimedServo extends Servo {

  private double m_angularSpeed = 0.2;
  private long m_completedTime = 0;
  private double m_currentAngle = 0.0;
  private double m_zero = 0.0;
  private boolean m_homed = false;
  private long m_travelTime = 0;
  private final boolean mDebug = true;

  /**
   * TimedServo constructur assuming zero angle is 0.0
   *
   * @param channel PWM port #
   * @param angularSpeed angular speed of the serve in degree/s
   */
  public TimedServo(int channel, double angularSpeed) {
    super(channel);
    this.m_angularSpeed = angularSpeed;
    this.m_homed = true;
  }

  /**
   * TimedServo constructur with specific zero angle
   *
   * @param channel PWM port #
   * @param angularSpeed angular speed of the serve in degree/s
   * @param zero angle in degrees which is the mechanical zero of this Servo
   */
  public TimedServo(int channel, double angularSpeed, double zero) {
    super(channel);
    this.m_angularSpeed = angularSpeed;
    this.m_currentAngle = zero;
    this.m_zero = zero;
  }

  @Override
  public void setAngle(double angle) {
    var delta = Math.abs(m_currentAngle - angle);

    var current_time = RobotController.getFPGATime();

    m_travelTime = (long) (delta / m_angularSpeed * Constants.kSecondsToMicroSeconds);
    m_completedTime = m_travelTime + current_time;
    super.setAngle(angle);
    if (mDebug)
      System.out.println(
          String.format(
              "Servo mapped on PWM %d going to %f degree from %f degree.\nCurrent time: %d completion time: %d",
              getChannel(), angle, m_currentAngle, current_time, m_completedTime));
    m_currentAngle = angle;
  }

  public void setZero() {
    if (m_homed) setAngle(m_zero);
    else reset();
  }

  private void reset() {
    if (mDebug)
      System.out.println(
          String.format("Rsetting Servo mapped on PWM %d at zero angle: %f", getChannel(), m_zero));
    super.setAngle(m_zero);
    m_currentAngle = m_zero;
    m_travelTime = (long) (180.0 / m_angularSpeed * Constants.kSecondsToMicroSeconds);
    m_completedTime = m_travelTime + RobotController.getFPGATime();
    this.m_homed = true;
  }

  /**
   * get the travel time
   *
   * @return total computed command travel time in seconds
   */
  public double travelTime() {
    var travTime = (double) m_travelTime / (double) Constants.kSecondsToMicroSeconds;
    if (mDebug) System.out.println(String.format("traveltime: %f", travTime));
    return travTime;
  }

  /**
   * Get the travel time given the completion ratio
   *
   * @param completionRatio from 0.0 to 1.0
   * @return travel time in seconds given the completion ratio or 0 if the command is completed
   */
  public double travelTime(double completionRatio) {
    if (!isDone())
      return (double) (completionRatio * (double) m_travelTime)
          / (double) Constants.kSecondsToMicroSeconds;
    return 0.0;
  }

  /**
   * Check if the last command has completed
   *
   * @return true if the command travel time is expired, false otherwise
   */
  public boolean isDone() {
    return RobotController.getFPGATime() > m_completedTime;
  }

  /**
   * Check if the previously executed command has completed up to a given ratio
   *
   * @param completionRatio from 0.0 to 1.0
   * @return true if the travel time is at least completionRatio passed
   */
  public boolean isDone(double completionRatio) {
    if (!isDone())
      return (m_completedTime - RobotController.getFPGATime()) > completionRatio * m_travelTime;
    return true;
  }
}
