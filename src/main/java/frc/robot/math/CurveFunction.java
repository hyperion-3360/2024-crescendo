package frc.robot.math;

public class CurveFunction{

  private double m_exponent;

  private double m_speed = 0.0;

  private double m_target = 0.0;

  public Double adjustPeriodic() {
    if (checkMotor() == false && m_speed < m_target){ //TODO epsilon ?
       System.out.println("speed curve " + m_speed);
        exponentialCurveMath();
        return m_speed;
       
    }

    return null;
  }

  private  void exponentialCurveMath() {
    m_exponent = m_exponent + 0.4;
     
    this.m_speed = this.m_speed * Math.exp(m_exponent);
  }

  public double getMotorSpeed(double target) {
    m_exponent = -2;
    m_target = target;

    exponentialCurveMath();

    return m_speed;
  }

  public void stop(){
    m_speed = 0.0;
  }

  private boolean checkMotor() {
    if (m_speed == 0.0) {
      return true;
    }
    return false;
  }

}
