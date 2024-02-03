package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.math.CurveFunction;

public class Shooter extends SubsystemBase {
  public enum shootSpeed {
    HIGH,
    LOW,
    INTAKE,
    TRAP,
    STOP
  }

  private static double highSpeed = 0.05;
  private static double lowSpeed = 0.05;
  private static double intakeSpeed = -0;
  private static double trapSpeed = 0;
  private static double stopSpeed = 0;

  private double m_speed = 0;

  private double m_shooterTargetSpeed;

  private CANSparkMax m_leftMaster = new CANSparkMax(
    Constants.SubsystemConstants.kShooterLeftMasterId, MotorType.kBrushless);
  private CANSparkMax m_rightMaster = new CANSparkMax(
    Constants.SubsystemConstants.kShooterRightMasterId, MotorType.kBrushless);
  private CANSparkMax m_leftFollower = new CANSparkMax(
    Constants.SubsystemConstants.kShooterLeftFollowerId, MotorType.kBrushless);
  private CANSparkMax m_rightFollower = new CANSparkMax(
    Constants.SubsystemConstants.kShooterRightFollowerId, MotorType.kBrushless);

    private CurveFunction m_curve = new CurveFunction();

  public Shooter() {

    m_rightMaster.restoreFactoryDefaults();
    m_rightMaster.setInverted(false);
    m_rightMaster.burnFlash();

    m_leftMaster.restoreFactoryDefaults();
    m_leftMaster.setInverted(true);
    m_leftMaster.burnFlash();

    m_leftFollower.follow(m_leftMaster, true);
    m_rightFollower.follow(m_rightMaster, false);
  }

  @Override
  public void periodic() {
    if (m_speed == 0.0) {
      m_curve.stop();
    } else {
      Double adjustedSpeed = m_curve.adjustPeriodic();
      if (adjustedSpeed != null) {
        m_rightMaster.set(adjustedSpeed);
      }
    }
  }

  private void setShootingLevel(shootSpeed shoot) {

    switch (shoot) {
      case LOW:
        m_speed = lowSpeed;
        m_shooterTargetSpeed = lowSpeed;
        break;

      case HIGH:
        m_speed = highSpeed;
        m_shooterTargetSpeed = highSpeed; 
        break;

      case STOP:
        m_speed = stopSpeed;
        m_shooterTargetSpeed = stopSpeed;
        break;

      case TRAP:
        m_speed = trapSpeed;
        m_shooterTargetSpeed = trapSpeed;
        break;

      case INTAKE:
        m_speed = intakeSpeed;
        m_shooterTargetSpeed = intakeSpeed;
        break;
    }
  }

  public void setShooterSpeed() {
    m_speed = m_curve.getMotorSpeed(m_shooterTargetSpeed);
    m_leftMaster.set(m_speed);
    m_rightMaster.set(m_speed);
  }

  public Command shoot(shootSpeed shootSpeed) {
    return this.runOnce(() -> setShootingLevel(shootSpeed))
        .andThen(new WaitCommand(5).andThen(() -> setShootingLevel(shootSpeed.STOP)));
  }
}
