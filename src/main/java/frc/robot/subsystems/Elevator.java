package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public enum e_elevatorLevel {
    HIGH,
    LOW,
    INTAKE
  };

  // instanciate a limit switch
  DigitalInput bottomlimitSwitch = new DigitalInput(5);

  // instancing the motor controllers m_elevatorLeft is the master motor
  private CANSparkMax m_elevatorRight =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_elevatorLeftMaster.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  private double m_elevatorRampRate = 0.2;

  private double kP = 2;
  private double kI = 0;
  private double kD = 0;
  private double m_voltage = 1100;

  private double m_speed = 0;

  private ProfiledPIDController m_PID =
      new ProfiledPIDController(kP, kI, kD, new Constraints(0.5, 0.5));

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

    m_encoder.setPosition(0.0);

    m_elevatorLeftMaster.setOpenLoopRampRate(m_elevatorRampRate);
    m_elevatorRight.setOpenLoopRampRate(m_elevatorRampRate);

    m_elevatorLeftMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorRight.setIdleMode(IdleMode.kBrake);
  }

  public void robotInit() {}

  @Override
  public void periodic() {
    setSpeed();

    m_elevatorLeftMaster.set(setSpeed());

    if (DriverStation.isDisabled()) {
      m_elevatorTarget = m_encoder.getPosition();
      m_PID.reset(m_encoder.getPosition());
      m_elevatorLeftMaster.set(0.0);

      System.out.println("ENCODER VALUE " + m_encoder.getPosition());
    }

    if (!bottomlimitSwitch.get()) {
      m_PID.reset(m_encoder.getPosition());
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

  // stops the motors
  public void stop() {
    m_elevatorLeftMaster.stopMotor();
  }

  public boolean onTarget() {

    return this.m_elevatorTarget <= m_encoder.getPosition();
  }

  public double setSpeed() {
    if (m_encoder.getPosition() - m_elevatorTarget >= 0) {
      return m_speed = -0.5;
    } else if (this.onTarget()) {
      return m_speed = 0.0;
    } else {
      return m_speed = m_speed * -1;
    }
  }

  public Command extendTheElevator(e_elevatorLevel m_elevatorLevel) {
    return this.runOnce(
        () -> {
          this.m_PID.reset(m_encoder.getPosition());
          this.setElevator(m_elevatorLevel);
        });
  }
}
