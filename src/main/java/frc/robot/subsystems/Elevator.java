package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.math.CurveFunction;

public class Elevator extends SubsystemBase {

  public enum e_elevatorLevel {
    HIGH,
    LOW,
    INTAKE
  };

  // DigitalInput bottomlimitSwitch = new DigitalInput(1);

  // instancing the motor controllers
  // private CANSparkMax m_elevatorRight = new
  // CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeft =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_elevatorLeft.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  private CurveFunction m_curve = new CurveFunction();

  // creating an elevator
  public Elevator() {
    // configures the CANSparkMax controllers
    m_elevatorLeft.restoreFactoryDefaults();
    // m_elevatorRight.restoreFactoryDefaults();
    m_elevatorLeft.setInverted(true);
    // m_elevatorLeft.follow(m_elevatorRight);

    m_encoder.setPosition(0.0);

  }

  public void robotInit() {}

  @Override
  public void periodic() {
    encoderConversions();

    if(this.onTarget()){
      this.stop();
    }else{
      Double adjustedSpeed = m_curve.adjustPeriodic();
      if(adjustedSpeed != null){
        m_elevatorLeft.set(adjustedSpeed);
      }
    }
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
    m_elevatorSpeed = m_curve.getMotorSpeed(m_elevatorSpeed);
    m_elevatorLeft.set(m_elevatorSpeed);
// m_elevatorRight.set(m_elevatorSpeed);
  }

  public void stop() {
    m_curve.stop();
    
          m_elevatorLeft.stopMotor();
          // m_elevatorRight.stopMotor();
  }

  // public void isAtBottom() {
  //   if(bottomlimitSwitch.get()) {
  //     }
  //   }

  private double encoderConversions() {
    double m_pulleyDiameter = 0.01445; //0.05445
    double m_beltRampUp = 0.0;
    while (m_encoder.getVelocity() > 0.0 == false) {
      /*add 0.00216 while the wheels are turning to the encoder wheel distance per pulse
         * this is because the belt is 0.00216 in thickness so we need to add it to the wheel
      circumference
         */
      m_beltRampUp = m_pulleyDiameter + 0.00216;
      return m_beltRampUp;
    }

    /*get the wheel distance per pulse
     * m_encoder.getPosition is the count per revolution of the encoder
     * 8.2 is the encoder rotation per mechanism rotation
     * pi * 0.05445 is the wheel circumference
     * the equation below is the wheel distance per pulse of the encoder
     */
    double m_encoderPosition =
        m_encoder.getPosition() / 12.2 * (Math.PI * m_pulleyDiameter) + m_beltRampUp;

    return m_encoderPosition;
  } 
  public boolean onTarget() {
    System.out.println("ENCODER VALUE: " + encoderConversions());
    return Math.abs(this.m_elevatorTarget - encoderConversions())
        < Constants.ElevatorConstants.kDeadzone;
  }

  //checks if the target is lower than the motors, if it is, lowers the motors
  private void goToTarget() {
    if (encoderConversions() > m_elevatorTarget) {
      setElevatorSpeed(-0.15);
    }else {
      setElevatorSpeed(0.15);
    }
  }

  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
    return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              this.setElevator(m_elevatorLevel);
              
            }),
            runOnce(() -> this.goToTarget()).until(this::onTarget)
      );
  }
}
