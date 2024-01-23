package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum elevatorLevel {
HIGH,
LOW,
INTAKE
};

public class Elevator extends SubsystemBase{

    DigitalInput topLimitSwitch = new DigitalInput(0);
    DigitalInput intakeLimitSwitch = new DigitalInput(1);
    DigitalInput lowLimitSwitch = new DigitalInput(2);
    
    private Spark m_elevatorRight = new Spark(Constants.SubsystemConstants.kelevatorRightId); 
    private Spark m_elevatorLeft = new Spark(Constants.SubsystemConstants.kelevatorLeftId); 
    
    // same as climber.java
    public void Elevator() {
        
        //configures the controllers
        m_elevatorLeft.setInverted(true);
        m_elevatorLeft.addFollower(m_elevatorRight);
    }
    
    public void robotInit()
    {
    Shuffleboard.getTab("Spark max").add("spark channel", m_elevatorLeft.getChannel());
    }
    
    @Override
    public void periodic()
    {

    }

public void setElevator(elevatorLevel m_elevatorLevel) {
      {
            switch (m_elevatorLevel) {
              case HIGH:
               
                break;
              case LOW:
                 
                break;
              case INTAKE:
               
                break;
            }
    }
  }
}
