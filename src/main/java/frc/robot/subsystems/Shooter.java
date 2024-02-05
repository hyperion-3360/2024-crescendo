package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  // different speed possibilities
  public enum shootSpeed {
    HIGH,
    LOW,
    INTAKE,
    TRAP,
    STOP
  }

  private static double highSpeed = 0.5; // requires testing
  private static double lowSpeed = 0.04; // requires testing
  private static double intakeSpeed = 0.4;
  private static double trapSpeed = 0; // requires testing
  private static double stopSpeed = 0;
  private static double rampRate = 6; // to be tuned according to battery and time consumption

  // declaring speed member
  private double m_speed = 0;

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
  }

  @Override
  public void periodic() {
    // setting speed to motors
    m_leftMaster.set(m_speed);
    m_rightMaster.set(m_speed);
  }

  private void setShootingSpeed(shootSpeed shoot) {

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

  /*
   * shooting to desired level except intake and stop
   */
  public Command shoot(shootSpeed shootSpeed) {
    return this.runOnce(() -> setShootingSpeed(shootSpeed))
        .andThen(new WaitCommand(5).andThen(this.stop()));
  }

  // stop the motors
  public Command stop() {
    return this.runOnce(() -> setShootingSpeed(shootSpeed.STOP));
  }

  // TODO: intake command that will have the infrared sensor to stop the spin
  public Command intake() {
    return this.run(() -> setShootingSpeed(shootSpeed.INTAKE)).until(this::hasNote).andThen(stop());
  }

  public boolean hasNote() {
    return !m_infraredSensor.get();
  }
}
