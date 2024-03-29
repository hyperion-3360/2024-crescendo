package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  public enum elevatorHeight {
    HIGH,
    FAR_HIGH_CLIMB,
    FAR_HIGH_AMP,
    LOW,
    INTAKE,
    AUTOFAR1,
    AUTOFAR2,
    AUTOFAR3,
    AUTOFAR4
  }

  // instanciate a limit switch
  DigitalInput bottomlimitSwitch = new DigitalInput(7);

  // instancing the motor controllers m_elevatorLeft is the master motor
  private CANSparkMax m_elevatorRightMaster =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorRightId, MotorType.kBrushless);
  private CANSparkMax m_elevatorLeft =
      new CANSparkMax(Constants.SubsystemConstants.kelevatorLeftId, MotorType.kBrushless);

  private RelativeEncoder m_encoder = m_elevatorLeft.getEncoder();
  private RelativeEncoder m_rightEncoder = m_elevatorRightMaster.getEncoder();

  // creating a target
  private double m_elevatorTarget = ElevatorConstants.kIntakeTarget;

  // creating the pid constants + pid member
  // private double kP = 0.09;
  // private double kI = 0.01;
  // private double kD = 0;
  private double kP = 0.12108;
  private double kI = 0.013;
  private double kD = 0;

  // these values come from the SysId routine (modified for set())
  // private double kS = 0.0008201;
  // private double kV = 0.0063509;
  // private double kA = 0.005103;
  // private double kG = 0.010439;

  // feedforward values for setVoltage()
  // private double kS = 0.05201;
  // private double kV = 0.063509;
  // private double kA = 0.0057103;
  // private double kG = 0.15439;

  private double kS = 0.3738;
  private double kV = 0.060294;
  private double kA = 0.0081298;
  private double kG = -0.36551;

  private String height = "intake";

  private PIDController m_pid = new PIDController(kP, kI, kD);
  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  private boolean m_sysIdEnable = false;

  // creating an elevator
  public Elevator() {
    // configures the CANSparkMax controllers
    m_elevatorLeft.restoreFactoryDefaults();
    m_elevatorRightMaster.restoreFactoryDefaults();

    m_elevatorLeft.setInverted(true);

    m_elevatorLeft.follow(m_elevatorRightMaster, true);

    m_elevatorLeft.setIdleMode(IdleMode.kBrake);
    m_elevatorRightMaster.setIdleMode(IdleMode.kBrake);

    if (!m_sysIdEnable) {
      m_elevatorLeft.setOpenLoopRampRate(0.6);
      m_elevatorRightMaster.setOpenLoopRampRate(0.6);
    }

    m_encoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);

    m_elevatorLeft.burnFlash();
    m_elevatorRightMaster.burnFlash();
  }

  @Override
  public void periodic() {
    // System.out.println(!bottomlimitSwitch.get());
    if (!m_sysIdEnable) {
      // calculate speed with pid
      m_elevatorRightMaster.setVoltage(
          m_feedforward.calculate(1)
              + m_pid.calculate(m_rightEncoder.getPosition(), m_elevatorTarget));

      // if the elevator touches the limit switch at the bottom of the rail set position to 0.0
      if (!bottomlimitSwitch.get()) {
        m_encoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);
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
  }

  // switch case statement for configuring elevator height
  private void setElevator(elevatorHeight m_elevatorLevel) {
    switch (m_elevatorLevel) {
      case HIGH:
        this.m_elevatorTarget = ElevatorConstants.kHighTarget;
        height = "Speaker";
        break;
      case FAR_HIGH_CLIMB:
        this.m_elevatorTarget = ElevatorConstants.kFarHighTarget;
        height = "Far shot from climb";
        break;
      case FAR_HIGH_AMP:
        this.m_elevatorTarget = ElevatorConstants.kFarHighTarget;
        height = "Far shot from amp";
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
        this.m_elevatorTarget = 56; // 65
        break;
      case AUTOFAR2:
        this.m_elevatorTarget = 58.5; // 67.5
        break;
      case AUTOFAR3:
        this.m_elevatorTarget = 53.5; // 62.5
        break;
      case AUTOFAR4:
        this.m_elevatorTarget = 50; // 60
        break;
    }
  }

  // stops the motors
  public void stop() {
    m_elevatorLeft.stopMotor();
    m_elevatorRightMaster.stopMotor();
  }

  // this is used for the leds in the sequences
  public boolean onTarget() {
    return m_rightEncoder.getPosition() >= this.m_elevatorTarget;
  }

  public void rawPosition(double target) {
    // Safety clamping
    if (target < Constants.ElevatorConstants.kIntakeTarget) {
      target = Constants.ElevatorConstants.kIntakeTarget;
    }
    if (target > Constants.ElevatorConstants.kHighTarget) {
      target = Constants.ElevatorConstants.kHighTarget;
    }

    this.m_elevatorTarget = target;
  }

  /**
   * Extend the elevator to a precise angle TODO: Make sure this gives valid values...
   *
   * @param angle angle to set the elevator in radians
   */
  public void extendTheElevator(double angle) {
    angle = angle / Math.PI * 180; // convert to degrees

    // clamp the target between the min and max valid values
    if (angle < Constants.ElevatorConstants.kMinElevatorAngle) {
      angle = Constants.ElevatorConstants.kMinElevatorAngle;
    }
    if (angle > Constants.ElevatorConstants.kMaxElevatorAngle) {
      angle = Constants.ElevatorConstants.kMaxElevatorAngle;
    }

    var angleToExtend = angle - Constants.ElevatorConstants.kMinElevatorAngle;

    final var valuesRange =
        Constants.ElevatorConstants.kHighTarget - Constants.ElevatorConstants.kIntakeTarget;
    final var anglesRange =
        Constants.ElevatorConstants.kMaxElevatorAngle
            - Constants.ElevatorConstants.kMinElevatorAngle;
    final var multFactor = valuesRange / anglesRange;

    var target = angleToExtend * multFactor + Constants.ElevatorConstants.kMinElevatorAngle;

    // Safety clamping
    if (target < Constants.ElevatorConstants.kIntakeTarget) {
      target = Constants.ElevatorConstants.kIntakeTarget;
    }
    if (target > Constants.ElevatorConstants.kHighTarget) {
      target = Constants.ElevatorConstants.kHighTarget;
    }

    this.m_elevatorTarget = target;
  }

  // extends the elevator to set target
  public Command extendTheElevator(elevatorHeight m_elevatorLevel) {
    return this.runOnce(() -> setElevator(m_elevatorLevel));
  }

  // returns current position (not used in anything YET)
  public double getCurrentPosition() {
    return m_encoder.getPosition();
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_angular_velocity =
      mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(null, null, null, null),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_elevatorRightMaster.setVoltage(volts.in(Volts));
                m_elevatorLeft.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                log.motor("elevator-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_elevatorLeft.getAppliedOutput() * m_elevatorLeft.getBusVoltage(),
                            Volts))
                    .angularPosition(m_angle.mut_replace(m_encoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_angular_velocity.mut_replace(
                            m_encoder.getVelocity() / 60.0, RotationsPerSecond));
                log.motor("elevator-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_elevatorRightMaster.getAppliedOutput()
                                * m_elevatorRightMaster.getBusVoltage(),
                            Volts))
                    .angularPosition(m_angle.mut_replace(m_rightEncoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_angular_velocity.mut_replace(
                            m_rightEncoder.getVelocity() / 60.0, RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
