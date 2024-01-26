
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase{
    
public enum e_elevatorLevel {
HIGH,
LOW,
INTAKE
};

  //DigitalInput bottomlimitSwitch = new DigitalInput(1);

    //instancing the motor controllers
    //private CANSparkMax m_elevatorRight = new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless); 
    private CANSparkMax m_elevatorLeft = new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless); 
    
Encoder m_encoder = new Encoder(9, 10, false, EncodingType.k2X);

    private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

    public e_elevatorLevel m_elevatorLevel;
    //creating an elevator
    public Elevator() {
        //configures the CANSparkMax controllers
         m_elevatorLeft.restoreFactoryDefaults();
       //m_elevatorRight.restoreFactoryDefaults();
        m_elevatorLeft.setInverted(true);
        //m_elevatorLeft.follow(m_elevatorRight);

        m_encoder.reset();
    }
    
    public void robotInit()
    {
      
    }
    
    @Override
    public void periodic()
    {

    }

private void setElevator(e_elevatorLevel m_elevatorLevel) {
            switch (m_elevatorLevel) {
              case HIGH:
              this.m_elevatorTarget = ElevatorConstants.kHighTarget;
                break;
              case LOW:
              this.m_elevatorTarget = ElevatorConstants.kLowTarget;
                break;
              case INTAKE:
               this.m_elevatorTarget = ElevatorConstants.kIntakeTarget;
                break;
            }
          }

public Command setElevatorSpeed(double m_elevatorSpeed) {
  return this.run(
    () -> {
     m_elevatorLeft.set(m_elevatorSpeed);
      //m_elevatorRight.set(m_elevatorSpeed);
    });
    }

public Command stop() {
  return this.runOnce(
  () -> {
  m_elevatorLeft.stopMotor();
  //m_elevatorRight.stopMotor();
    });
    }
    
// public void isAtBottom() {
//   if(bottomlimitSwitch.get()) {
//  m_encoder.reset();
//     }
//   }

public double setelevatorAngleCorrection() {
  //pulley diameter
  double m_elevatorAngle = 0.05445;
  // TODO fine tune those values
  // while the encoder isn't stopped increase the diameter because of the belt
  while (m_encoder.getStopped() == false) {
    m_elevatorAngle = m_elevatorAngle + 0.00216;
  }
  //TODO add aprilTag math when aprilTag done
  return m_elevatorAngle;
  }                                                                                  //5,445cm + 2,16 mm these are the pulley diameter and belt thickness respectively

  public boolean onTarget() {
    return Math.abs(m_elevatorTarget - setelevatorAngleCorrection())
     < Constants.ElevatorConstants.kDeadzone;
  }

  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
      return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              this.setElevator(m_elevatorLevel);
            }),
        this.run(() -> this.setElevator(m_elevatorLevel)).until(this::onTarget));
      }

      public Command woerk() {
        return this.runOnce(() -> {
        System.out.println("woerk");
        });
      }

}