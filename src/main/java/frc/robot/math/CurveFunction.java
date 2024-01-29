package frc.robot.math;

public class CurveFunction{

  private enum State{
    RUNNING,
    STOPPED
  }
  private State m_state = State.STOPPED;

  private double m_exponent;

  private double m_speed = 0.0;

  private double m_target = 0.0;

  private double m_eps = 0.001;

  public Double adjustPeriodic() {
    //checks if the exponential function has reached the target or including if the speed is negative
    if (isRunning() && (m_speed < (m_target - m_eps) ||
      (m_speed < 0 && m_speed > (m_target + m_eps))))
     {
        exponentialCurveMath();
        return m_speed;
       
    }

    return null;
  }

  private  void exponentialCurveMath() {
    m_exponent = m_exponent + 0.4;
     
    this.m_speed = this.m_target * Math.exp(m_exponent);
  }

  public double getMotorSpeed(double target) {
    m_exponent = -2;
    m_target = target;
    m_state = State.RUNNING;

    exponentialCurveMath();

    return m_speed;
  }

  public void stop(){
    m_speed = 0.0;
    m_state = State.STOPPED;
  }

  private boolean isRunning() {
    return m_state == State.RUNNING;
  }

}
