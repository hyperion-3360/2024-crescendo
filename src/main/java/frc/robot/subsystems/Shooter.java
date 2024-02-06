package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public enum shootSpeed {
    HIGH,
    LOW,
    INTAKE,
    TRAP,
    STOP
  }

  private static double highSpeed = 0.5; // 55% seem good
  private static double lowSpeed = 0.04;
  private static double intakeSpeed = 0.3;
  private static double trapSpeed = 0;
  private static double stopSpeed = 0;
  private static double rampRate = 5;

  private double m_speed = 0;

  private CANSparkMax m_leftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kLeftMasterId, MotorType.kBrushless);
  private CANSparkMax m_rightMaster =
      new CANSparkMax(Constants.SubsystemConstants.kRightMasterId, MotorType.kBrushless);
  private CANSparkMax m_leftFollower =
      new CANSparkMax(Constants.SubsystemConstants.kLeftFollowerId, MotorType.kBrushless);
  private CANSparkMax m_rightFollower =
      new CANSparkMax(Constants.SubsystemConstants.kRightFollowerId, MotorType.kBrushless);

  public Shooter() {

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
    // calcSpeed();
    m_leftMaster.set(m_speed);
    m_rightMaster.set(m_speed);

    // System.out.println("speed " + m_leftMaster.getAppliedOutput());
  }

  private void setShootingLevel(shootSpeed shoot) {

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

  public Command shoot(shootSpeed shootSpeed) {
    return this.runOnce(() -> setShootingLevel(shootSpeed))
        .andThen(new WaitCommand(5).andThen(this.stop()));
  }

  public Command stop() {
    return this.runOnce(() -> setShootingLevel(shootSpeed.STOP));
  }
}
