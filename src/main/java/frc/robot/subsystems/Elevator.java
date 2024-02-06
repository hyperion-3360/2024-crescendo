package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.math.Conversions;

public class Elevator extends SubsystemBase {

  public enum e_elevatorLevel {
    HIGH,
    LOW,
    INTAKE
  };

  // instanciate a limit switch
  DigitalInput m_bottomlimitSwitch = new DigitalInput(5);

  // instancing the motor controllers m_elevatorLeft is the master motor
  private CANSparkMax m_elevatorRight =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);
  // calls the integrated Hall sensor relative encoder for the NEO 550
  private RelativeEncoder m_encoderLeft = m_elevatorLeftMaster.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  private double m_elevatorRampRate = 0.2;

  // creating an elevator
  public Elevator() {
    // configures the CANSparkMax controllers
    m_elevatorLeftMaster.restoreFactoryDefaults();
    m_elevatorRight.restoreFactoryDefaults();

    m_elevatorLeftMaster.setInverted(true);

    m_elevatorRight.follow(m_elevatorLeftMaster, true);

    m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 3);
    m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 3);

    m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 6);
    m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 6);

    m_encoderLeft.setPosition(0.0);

    m_elevatorLeftMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorRight.setIdleMode(IdleMode.kBrake);

    m_elevatorLeftMaster.setOpenLoopRampRate(m_elevatorRampRate);
    m_elevatorRight.setOpenLoopRampRate(m_elevatorRampRate);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_elevatorLeftMaster.stopMotor();
      m_elevatorTarget = m_encoderLeft.getPosition();
    }

    if (!m_bottomlimitSwitch.get()) {
      m_encoderLeft.setPosition(0.0);
    }

    encoderConversion();
  }

  // switch case statement for configuring elevator height
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

  // setting the elevator speed according to the exponential function
  public double setElevatorSpeed() {
    double m_elevatorSpeed = m_elevatorLeftMaster.get();
    m_elevatorLeftMaster.set(m_elevatorSpeed);
    return m_elevatorSpeed;
  }

  // stops the motors
  public void stop() {
    m_elevatorLeftMaster.stopMotor();
  }

  /*
   * 35 = gear ratio.
   * 0.041275 = pulley diameter. belt size = 0.00182
   * encoder position is the position of the encoder
   */
  private double encoderConversion() {
    double m_encoderPosition;
    m_encoderPosition = Conversions.NEOToMeters(35, 0.041275, m_encoderLeft.getPosition());
    return m_encoderPosition;
  }

  // checks if the elevator has reached the target
  public boolean onTarget() {
    return Math.abs(this.m_elevatorTarget - encoderConversion())
        < Constants.ElevatorConstants.kDeadzone;
  }

  private double negativeTargetChecker() {
    if (encoderConversion() < m_elevatorTarget) {
      return setElevatorSpeed();
    }
    return (setElevatorSpeed() - 0.3) * -1;
  }

  // checks if the target is lower than the elevator, if it is, lowers the elevator

  // a command to extend the elevator
  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
    return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              this.setElevator(m_elevatorLevel);
              negativeTargetChecker();
            }),
        run(() -> this.m_elevatorLeftMaster.set(0.5))
            .until(this::onTarget)
            .andThen(
                run(
                    () -> {
                      m_elevatorLeftMaster.set(0.0);
                    })));
  }

  public void robotInit() {}
}
