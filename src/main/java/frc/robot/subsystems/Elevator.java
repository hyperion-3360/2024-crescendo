package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public enum elevatorHeight {
    HIGH,
    FAR_HIGH,
    LOW,
    INTAKE,
    AUTOFAR1,
    AUTOFAR2,
    AUTOFAR3
  }

  // instanciate a limit switch
  DigitalInput bottomlimitSwitch = new DigitalInput(7);

  // instancing the motor controllers m_elevatorLeft is the master motor
  private CANSparkMax m_elevatorRight =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_elevatorLeftMaster.getEncoder();

  // creating a target
  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  // creating the pid constants + pid member
  private double kP = 0.0075;
  private double kI = 0.00083;
  private double kD = 0;

  private String height = "no data";

  private PIDController m_pid = new PIDController(kP, kI, kD);

  // creating an elevator
  public Elevator() {
    // configures the CANSparkMax controllers
    m_elevatorLeftMaster.restoreFactoryDefaults();
    m_elevatorRight.restoreFactoryDefaults();

    m_elevatorLeftMaster.setInverted(true);

    m_elevatorRight.follow(m_elevatorLeftMaster, true);

    m_elevatorLeftMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorRight.setIdleMode(IdleMode.kBrake);

    m_elevatorLeftMaster.setOpenLoopRampRate(2);
    m_elevatorRight.setOpenLoopRampRate(2);

    m_encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {

    // calculate speed with pid
    m_elevatorLeftMaster.set(m_pid.calculate(m_encoder.getPosition(), m_elevatorTarget));

    // if the elevator touches the limit switch at the bottom of the rail set position to 0.0
    if (!bottomlimitSwitch.get()) {
      m_encoder.setPosition(0.0);
      m_pid.reset();
    }

    if (DriverStation.isDisabled()) {
      extendTheElevator(elevatorHeight.HIGH).cancel();
      extendTheElevator(elevatorHeight.LOW).cancel();
      extendTheElevator(elevatorHeight.INTAKE).cancel();
      m_pid.reset();
      m_elevatorTarget = 0.0;
    }
    SmartDashboard.putString("Elevator Target", height);
  }

  // switch case statement for configuring elevator height
  private void setElevator(elevatorHeight m_elevatorLevel) {
    switch (m_elevatorLevel) {
      case HIGH:
        this.m_elevatorTarget = ElevatorConstants.kHighTarget;
        height = "Speaker";
        break;
      case FAR_HIGH:
        this.m_elevatorTarget = ElevatorConstants.kFarHighTarget;
        height = "Far Speaker";
        break;
      case LOW:
        this.m_elevatorTarget = ElevatorConstants.kLowTarget;
        height = "Amp";
        break;
      case INTAKE:
        this.m_elevatorTarget = ElevatorConstants.kIntakeTarget;
        height = "Intake";
        break;
      case AUTOFAR1:
        this.m_elevatorTarget = 65;
        break;
      case AUTOFAR2:
        this.m_elevatorTarget = 39.5;
        break;
      case AUTOFAR3:
        this.m_elevatorTarget = 39.5;
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

  // extends the elevator to set target
  public Command extendTheElevator(elevatorHeight m_elevatorLevel) {
    return this.runOnce(() -> setElevator(m_elevatorLevel));
  }

  // returns current position (not used in anything YET)
  public double getCurrentPosition() {
    return m_encoder.getPosition();
  }
}
