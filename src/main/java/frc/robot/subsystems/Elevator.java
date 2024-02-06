package frc.robot.subsystems;

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
import frc.robot.math.CurveFunction;

public class Elevator extends SubsystemBase {

  public enum e_elevatorLevel {
    HIGH,
    LOW,
    INTAKE
  };

  // instanciate a limit switch
  DigitalInput bottomlimitSwitch = new DigitalInput(5);

  // instancing the motor controllers m_elevatorLeft is the master motor
  // private CANSparkMax m_elevatorRight =
  //     new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);
  // calls the integrated Hall sensor relative encoder for the NEO 550
  private RelativeEncoder m_encoderLeft = m_elevatorLeftMaster.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;
  // useful exponential curve function to extend motor lifespan
  private CurveFunction m_curve = new CurveFunction();

  // creating an elevator
  public Elevator() {
    // configures the CANSparkMax controllers
    m_elevatorLeftMaster.restoreFactoryDefaults();
    // m_elevatorRight.restoreFactoryDefaults();

    m_elevatorLeftMaster.setInverted(true);

    // m_elevatorRight.follow(m_elevatorLeftMaster, true);

    m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 3);
    // m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 3);

    m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 6);
    // m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 6);

    m_encoderLeft.setPosition(0.0);
    // m_encoderRight.setPosition(0.0);

    m_elevatorLeftMaster.set(0.6);

    m_encoderLeft.setPositionConversionFactor(0.1);
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      m_elevatorTarget = m_encoderLeft.getPosition();
    }
    m_elevatorLeftMaster.set(0.0);

    // checks if the motor is running or at max speed for the exponential function
    if (this.onTarget()) {
      this.stop();
    } else {
      Double adjustedSpeed = m_curve.adjustPeriodic();
      if (adjustedSpeed != null) {
        m_elevatorLeftMaster.set(adjustedSpeed);
      }
    }

    if (isAtBottom()) {
      m_encoderLeft.setPosition(0.0);
    }
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
  public double setElevatorSpeed(double m_elevatorSpeed) {
    m_elevatorSpeed = m_curve.getMotorSpeed(m_elevatorSpeed);
    m_elevatorLeftMaster.set(m_elevatorSpeed);
    return m_elevatorSpeed;
  }

  // stops the motors
  public void stop() {
    m_curve.stop();

    m_elevatorLeftMaster.stopMotor();
  }

  /*
   * 35 = gear ratio.
   * 0.5445 = pulley diameter.
   * encoder position is the position of the encoder
   */
  private double encoderConversion() {
    double m_encoderPosition;
    m_encoderPosition = Conversions.NEOToMeters(35, 0.5445, m_encoderLeft.getPosition());
    return m_encoderPosition;
  }

  // check if the limit switch is triggered
  public boolean isAtBottom() {
    if (bottomlimitSwitch.get()) {
      return true;
    }
    return false;
  }

  // checks if the elevator has reached the target
  public boolean onTarget() {
    return Math.abs(this.m_elevatorTarget - encoderConversion())
        < Constants.ElevatorConstants.kDeadzone;
  }

  // checks if the target is lower than the elevator, if it is, lowers the elevator
  private void goToTarget() {
    if (encoderConversion() < m_elevatorTarget) {
      setElevatorSpeed(0.30);

    } else {
      setElevatorSpeed(-0.20);
    }
  }

  // a command to extend the elevator
  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
    return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              this.setElevator(m_elevatorLevel);
            }),
        run(() -> this.goToTarget())
            .until(this::onTarget)
            .andThen(
                run(
                    () -> {
                      m_elevatorLeftMaster.set(0.03);
                    })));
  }

  public void robotInit() {}
}
