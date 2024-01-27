
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

    private double m_encoderPosition = m_encoder.getDistancePerPulse();
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

public void setElevatorSpeed(double m_elevatorSpeed) {
     m_elevatorLeft.set(m_elevatorSpeed);
      //m_elevatorRight.set(m_elevatorSpeed);
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

private double encoderConversions() {
  double m_pulleyDiameter = 0.05445;
  double m_beltRampUp = 0.0;
  while (m_encoder.getStopped() == false) {
  /*add 0.00216 while the wheels are turning to the encoder wheel distance per pulse 
   * this is because the belt is 0.00216 in thickness so we need to add it to the wheel circumference
   */
    m_beltRampUp = m_pulleyDiameter + 0.00216;
    return m_beltRampUp;
  }

  /*get the wheel distance per pulse
   * 1/42 is the ticks per revolution of the encoder
   * 8.2 is the encoder rotation per mechanism rotation
   * pi * 0.05445 is the wheel circumference
   * the equation below is the wheel distance per pulse of the encoder
  */
  m_encoderPosition = (1/ 42)/(8.2) * Math.PI * m_pulleyDiameter + m_beltRampUp;

  return m_encoderPosition;
}                                                                     //5,445cm + 2,16 mm these are the pulley diameter and belt thickness respectively

  public boolean onTarget() {
    return Math.abs(this.m_elevatorTarget - encoderConversions())
     < Constants.ElevatorConstants.kDeadzone;
  }

  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
      return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              this.setElevator(m_elevatorLevel);
            }),
        this.run(() -> this.setElevatorSpeed(0.05)).until(this::onTarget));
      }
    
}