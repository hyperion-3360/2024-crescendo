package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public enum elevatorHeight {
    HIGH,
    LOW,
    INTAKE
  };

  // instanciate a limit switch
  DigitalInput bottomlimitSwitch = new DigitalInput(7);

  // instancing the motor controllers m_elevatorLeft is the master motor
  private CANSparkMax m_elevatorRight =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_elevatorLeftMaster.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  private double kP = 0.013;
  private double kI = 0;
  private double kD = 0;

  private PIDController m_pid = new PIDController(kP, kI, kD);

  // private final ProfiledPIDController m_pid =
  // new ProfiledPIDController(kp, 0.0 ,0.0, new Constraints(m_velocity, m_acceleration));

  // just in case
  // private double m_pulleyDiameter = 0.05445;

  // private double m_beltRampUp = 0.0;

  // creating an elevator
  public Elevator() {
    // configures the CANSparkMax controllers
    m_elevatorLeftMaster.restoreFactoryDefaults();
    m_elevatorRight.restoreFactoryDefaults();

    m_elevatorLeftMaster.setInverted(true);

    m_elevatorRight.follow(m_elevatorLeftMaster, true);

    // m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    // m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    // m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    // m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    m_elevatorLeftMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorRight.setIdleMode(IdleMode.kBrake);

    m_elevatorLeftMaster.setOpenLoopRampRate(2);
    m_elevatorRight.setOpenLoopRampRate(2);

    m_encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {

    m_elevatorLeftMaster.set(m_pid.calculate(m_encoder.getPosition(), m_elevatorTarget));

    if (!bottomlimitSwitch.get()) {
      m_encoder.setPosition(0.0);
    }

    //    System.out.println(isHigh());
  }

  // switch case statement for configuring elevator height
  private void setElevator(elevatorHeight m_elevatorLevel) {
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

  // stops the motors
  public void stop() {

    m_elevatorLeftMaster.stopMotor();
  }

  // this is used for the leds in the sequences
  public boolean onTarget() {

    return m_encoder.getPosition() >= this.m_elevatorTarget;
  }

  public Command extendTheElevator(elevatorHeight m_elevatorLevel) {
    return this.runOnce(() -> setElevator(m_elevatorLevel));
  }

  public double getCurrentPosition() {
    return m_encoder.getPosition();
  }

  public boolean isHigh() {
    if (m_elevatorTarget == Constants.ElevatorConstants.kHighTarget) {
      return true;
    } else return false;
  }

  public boolean isLow() {
    if (m_elevatorTarget == Constants.ElevatorConstants.kLowTarget) {
      return true;
    } else return false;
  }
}
