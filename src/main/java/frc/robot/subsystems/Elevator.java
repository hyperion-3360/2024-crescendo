
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
    
    private CANSparkMax m_elevatorRight = new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless); 
    private CANSparkMax m_elevatorLeft = new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless); 
    
    private double m_elevatorTarget = ;

    // same as climber.java
    public void Elevator() {
        
        //configures the controllers
        m_elevatorLeft.restoreFactoryDefaults();
        m_elevatorRight.restoreFactoryDefaults();
        m_elevatorLeft.setInverted(true);
        m_elevatorLeft.follow(m_elevatorRight);
    }
    
    public void robotInit()
    {
  
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
