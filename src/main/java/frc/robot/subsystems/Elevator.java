
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

enum elevatorLevel {
HIGH,
LOW,
INTAKE
};

public class Elevator extends SubsystemBase{
    
    //instancing the motor controllers
    private CANSparkMax m_elevatorRight = new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless); 
    private CANSparkMax m_elevatorLeft = new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless); 
    
    private double m_elevatorTarget;

    //creating an elevator
    public void Elevator() {
        
        //configures the CANSparkMax controllers
        m_elevatorLeft.restoreFactoryDefaults();
        m_elevatorRight.restoreFactoryDefaults();
        m_elevatorLeft.setInverted(true);
        m_elevatorLeft.follow(m_elevatorRight);
        //m_elevatorTarget = ; // TODO set the elevator target to an meter unit
    }
    
    public void robotInit()
    {
    }
    
    @Override
    public void periodic()
    {

    }

public void setElevator(elevatorLevel m_elevatorLevel) {
      
            switch (m_elevatorLevel) {
              case HIGH:
              m_elevatorTarget = ElevatorConstants.khighTarget;
                break;
              case LOW:
              m_elevatorTarget = ElevatorConstants.klowTarget;
                break;
              case INTAKE:
               m_elevatorTarget = ElevatorConstants.kintakeTarget;
                break;
                default: m_elevatorTarget = ElevatorConstants.kintakeTarget;
            }
          }

public void setElevatorSpeed(double m_elevatorSpeed) {
      m_elevatorLeft.set(m_elevatorSpeed);
      m_elevatorRight.set(m_elevatorSpeed);
    }

public Command stop() {
  return this.runOnce(
  () -> {
  m_elevatorRight.stopMotor();
    
    });
    }
    

  }

