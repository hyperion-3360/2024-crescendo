package frc.lib.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class TimedServo extends Servo {

  private double m_angularSpeed = 0.0;
  private double m_completedTime = 0.0;


  public TimedServo(int channel, double angularSpeed){

    super(channel);
    this.m_angularSpeed = angularSpeed;

  }
  
  @Override
  public void setAngle(double degrees){

    m_completedTime = degrees * m_angularSpeed + RobotController.getFPGATime();
    new RunCommand(()-> super.setAngle(degrees)).until(()->this.isDone());

  }

public boolean isDone(){

  return RobotController.getFPGATime() > m_completedTime;
}
}
