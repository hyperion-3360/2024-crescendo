package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // different speed possibilities
  public enum levelSpeed {
    HIGH,
    LOW,
    INTAKE,
    TRAP,
    STOP
  }

  private static double highSpeed = 0.8; // need to add perk to adjust speed according to distance
  private static double lowSpeed = 0.3; // requires testing
  private static double intakeSpeed = 0.4;
  private static double trapSpeed = 0.15; // requires testing
  private static double stopSpeed = 0;
  private static double rampRate = 1; // to be tuned according to battery and time consumption

  // declaring speed member
  private double m_speed = 0;
  private levelSpeed m_targetLevel = levelSpeed.STOP;

  // blocker constants
  private final double kIntakeHookAngleOpen = 0.0;
  private final double kIntakeHookAngleClose = 23;

  // declaring motors for the shooter
  private CANSparkMax m_leftMaster =
      new CANSparkMax(Constants.ShooterConstants.kLeftMasterId, MotorType.kBrushless);
  private CANSparkMax m_rightMaster =
      new CANSparkMax(Constants.ShooterConstants.kRightMasterId, MotorType.kBrushless);
  private CANSparkMax m_leftFollower =
      new CANSparkMax(Constants.ShooterConstants.kLeftFollowerId, MotorType.kBrushless);
  private CANSparkMax m_rightFollower =
      new CANSparkMax(Constants.ShooterConstants.kRightFollowerId, MotorType.kBrushless);

  // infrared sensor for intake
  private DigitalInput m_infraredSensor =
      new DigitalInput(Constants.ShooterConstants.kInfraredSensorId);

  // creating the blocker servo
  private Servo m_blocker = new Servo(Constants.ShooterConstants.kservoBlockerId);

  public Shooter() {

    // configs
    m_leftMaster.restoreFactoryDefaults();
    m_rightMaster.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();

    m_leftMaster.setInverted(true);
    m_leftFollower.setInverted(true);

    m_leftMaster.setIdleMode(IdleMode.kCoast);
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightMaster.setIdleMode(IdleMode.kCoast);
    m_rightFollower.setIdleMode(IdleMode.kCoast);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    // the openLoopRampRate is a feature from spark max that caps the maximum acceleration from 0 to
    // +/- 1 in seconds
    m_rightMaster.setOpenLoopRampRate(rampRate);
    m_leftMaster.setOpenLoopRampRate(rampRate);
    m_rightFollower.setOpenLoopRampRate(rampRate);
    m_leftFollower.setOpenLoopRampRate(rampRate);

    m_rightMaster.burnFlash();
    m_leftFollower.burnFlash();
    m_rightFollower.burnFlash();
    m_leftMaster.burnFlash();

    // setting the blocker to closed on boot
    m_blocker.setAngle(kIntakeHookAngleClose);
  }

  @Override
  public void periodic() {
    // setting speed to motors
    m_leftMaster.set(m_speed);
    m_rightMaster.set(m_speed);
    SmartDashboard.putBoolean("Has Note?", hasNote());
  }

  // switch case for different speeds according to the level
  private void setSpeedFor(levelSpeed shoot) {

    switch (shoot) {
      case LOW:
        m_speed = lowSpeed;
        break;

      case HIGH:
        m_speed = highSpeed;
        break;

      case STOP:
        m_speed = stopSpeed;
        break;

      case TRAP:
        m_speed = trapSpeed;
        break;

      case INTAKE:
        m_speed = intakeSpeed;
        break;
    }
  }

  // command to hold speed according to the target level
  public Command holdSpeed(levelSpeed level) {
    return this.run(() -> this.setSpeedFor(level));
  }

  // stop the motors
  public Command stop() {
    return this.runOnce(() -> this.setSpeedFor(levelSpeed.STOP));
  }

  // shoot to desired level
  public Command shootTo() {
    return Commands.sequence(
        this.hookIntake(),
        this.setSpeedWithTarget(),
        new WaitCommand(1),
        this.hookRelease(),
        new WaitCommand(0.7),
        this.stop(),
        this.hookIntake());
  }

  // setting the target level
  public Command setTargetLevel(levelSpeed level) {
    return this.runOnce(() -> m_targetLevel = level);
  }

  // set the speed according to the target
  public Command setSpeedWithTarget() {
    return this.runOnce(() -> this.setSpeedFor(m_targetLevel));
  }

  // set hook to intake mode, then set target + speed until has note
  public Command intake() {
    return Commands.sequence(
        this.hookIntake(), this.holdSpeed(levelSpeed.INTAKE).until(this::hasNote), this.stop());
  }

  // infrared sensor to see if we have a note
  public boolean hasNote() {
    return !m_infraredSensor.get();
  }

  // set down the hook
  public Command hookIntake() {
    return this.runOnce(() -> m_blocker.setAngle(kIntakeHookAngleClose));
  }

  // set up the hook
  public Command hookRelease() {
    return this.runOnce(() -> m_blocker.setAngle(kIntakeHookAngleOpen));
  }
}
