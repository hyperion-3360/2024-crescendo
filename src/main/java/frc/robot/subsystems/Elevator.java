ackage frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.swerve.Swerve;

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

  private RelativeEncoder m_encoder = m_elevatorLeftMaster.getEncoder();

  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  private double m_elevatorRampRate = 0.2;

  private Swerve m_swerve = new Swerve();

  private AprilTag m_aprilTag = new AprilTag(0, null);

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

    m_elevatorLeftMaster.setIdleMode(IdleMode.kBrake);
    m_elevatorRight.setIdleMode(IdleMode.kBrake);

    m_elevatorLeftMaster.setOpenLoopRampRate(m_elevatorRampRate);
    m_elevatorRight.setOpenLoopRampRate(m_elevatorRampRate);
  }

  public void robotInit() {}

  @Override
  public void periodic() {
    encoderConversions();

    if (DriverStation.isDisabled()) {
      m_elevatorTarget = m_encoder.getPosition();
    }
    m_elevatorLeftMaster.set(0.0);

    if (!m_bottomlimitSwitch.get()) {
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
    m_elevatorLeftMaster.set(m_elevatorSpeed);
  }

  // stops the motors
  public void stop() {
    m_elevatorLeftMaster.stopMotor();
  }

  private double encoderConversions() {

    double m_encoderPosition = m_encoder.getPosition() / 360;
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

  private void foundDiagonal() {
    var m_foundDiagonal = 0.0;
    double xposition = m_swerve.m_odometry.getPoseMeters().getX();
    double yPosition = m_swerve.m_odometry.getPoseMeters().getY();
    xposition = Math.pow(xposition, 2);
    yPosition = Math.pow(yPosition, 2);
    var cPosition = xposition + yPosition;

    m_foundDiagonal = Math.sqrt(cPosition);
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
