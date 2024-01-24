
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
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
    
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

    //instancing the motor controllers
    private CANSparkMax m_elevatorRight = new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless); 
    private CANSparkMax m_elevatorLeft = new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless); 
    
    private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

    //creating an elevator
    public Elevator() {
        //configures the CANSparkMax controllers
         m_elevatorLeft.restoreFactoryDefaults();
       m_elevatorRight.restoreFactoryDefaults();
        m_elevatorLeft.setInverted(true);
        m_elevatorLeft.follow(m_elevatorRight);
    }
    
    public void robotInit()
    {;
      //SmartDashboard.putData("motor test", setElevatorSpeedmmmm(0.1));
      
    }
    
    @Override
    public void periodic()
    {

    }

public void setElevator(elevatorLevel m_elevatorLevel) {
      
            switch (m_elevatorLevel) {
              case HIGH:
              this.m_elevatorTarget = ElevatorConstants.kHighTarget + 
              setelevatorAngleFineTuning(0.1);
                break;
              case LOW:
              this.m_elevatorTarget = ElevatorConstants.kLowTarget + 
              setelevatorAngleFineTuning(0.1);
                break;
              case INTAKE:
               this.m_elevatorTarget = ElevatorConstants.kIntakeTarget;
                break;
                default: this.m_elevatorTarget = ElevatorConstants.kIntakeTarget;
            }
          }

public void setElevatorSpeed(double m_elevatorSpeed) {
      m_elevatorLeft.set(m_elevatorSpeed);
      m_elevatorRight.set(m_elevatorSpeed);
    }

    public void setElevatorSpeedmmmm(double m_elevatorSpeedmmm) {
      m_elevatorLeft.set(m_elevatorSpeedmmm);
      };
    

public Command stop() {
  return this.runOnce(
  () -> {
  m_elevatorLeft.stopMotor();
  m_elevatorRight.stopMotor();
    });
    }
    
public void isAtBottom() {
  if(bottomlimitSwitch.get()) {
  m_elevatorLeft.restoreFactoryDefaults();
  m_elevatorRight.restoreFactoryDefaults();
    }
  }

public double setelevatorAngleFineTuning(double m_elevatorAngle) {
  //TODO add aprilTag math when aprilTag done
  return m_elevatorAngle ;
}

}

