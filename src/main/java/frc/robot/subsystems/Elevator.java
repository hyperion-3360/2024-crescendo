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
import frc.robot.math.CurveFunction;

public class Elevator extends SubsystemBase {

  public enum e_elevatorLevel {
    HIGH,
    LOW,
    INTAKE
  };

  // instanciate a limit switch
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  // instancing the motor controllers m_elevatorLeft is the master motor
  private CANSparkMax m_elevatorRight =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_elevatorLeftMaster.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  private CurveFunction m_curve = new CurveFunction();

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

    m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    m_elevatorLeftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    m_elevatorRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);

    m_encoder.setPosition(0.0);
  }

  public void robotInit() {}

  @Override
  public void periodic() {
    encoderConversions();

    if (DriverStation.isDisabled()) {
      m_elevatorTarget = m_encoder.getPosition();
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
      m_encoder.setPosition(0.0);
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
  public void setElevatorSpeed(double m_elevatorSpeed) {
    m_elevatorSpeed = m_curve.getMotorSpeed(m_elevatorSpeed);
    m_elevatorLeftMaster.set(m_elevatorSpeed);
  }

  // stops the motors
  public void stop() {
    m_curve.stop();

    m_elevatorLeftMaster.stopMotor();
  }

  // check if the limit switch is triggered
  public boolean isAtBottom() {
    if (bottomlimitSwitch.get()) {

      return true;
    }
    return false;
  }

  private double encoderConversions() {

    double m_encoderPosition =
        m_encoder.getPosition() / 360; // * 1/m_encoder.getCountsPerRevolution() *
    // m_pulleyDiameter + m_beltRampUp * 3.14159265358979323846;

    return m_encoderPosition;
  }

  private boolean negativeTargetChecker() {
    if (encoderConversions() < m_elevatorTarget) {
      return true;
    }
    return false;
  }

  public boolean onTarget() {

    if (negativeTargetChecker() == false) {

      return Math.abs(this.m_elevatorTarget + encoderConversions())
          < Constants.ElevatorConstants.kDeadzone;
    }
    return Math.abs(this.m_elevatorTarget - encoderConversions())
        < Constants.ElevatorConstants.kDeadzone;
  }

  // checks if the target is lower than the motors, if it is, lowers the motors
  private void goToTarget() {
    if (encoderConversions() < m_elevatorTarget) {
      setElevatorSpeed(0.50);

    } else {
      setElevatorSpeed(-0.20);
    }
  }

  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
    return new SequentialCommandGroup(
        this.runOnce(
            () -> {
              // this.m_pid.reset(m_encoder.getPosition());
              this.setElevator(m_elevatorLevel);
            }),
        run(() -> this.goToTarget())
            .until(this::onTarget)
            .andThen(run(() -> m_elevatorLeftMaster.set(0.03))));
  }
}
