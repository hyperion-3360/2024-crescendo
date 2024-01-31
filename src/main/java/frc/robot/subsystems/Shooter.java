package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum shoot {
  HIGH,
  LOW
}

public class Shooter extends SubsystemBase {

  // Create new motors

  private TalonSRX m_RightMaster = new TalonSRX(Constants.ShooterConstants.kRightMasterid);
  private TalonSRX m_RightFollower = new TalonSRX(Constants.ShooterConstants.kRightFollowerid);
  private TalonSRX m_LeftMaster = new TalonSRX(Constants.ShooterConstants.kLeftMasterid);
  private TalonSRX m_LeftFollower = new TalonSRX(Constants.ShooterConstants.kLeftFollowerid);

  public void Shooter() {

    // Config motors

    m_LeftMaster.setInverted(true);
    m_LeftFollower.setInverted(true);

    m_RightMaster.configFactoryDefault();
    m_RightFollower.configFactoryDefault();
    m_LeftMaster.configFactoryDefault();
    m_LeftFollower.configFactoryDefault();

    m_LeftFollower.follow(m_LeftMaster);
    m_RightFollower.follow(m_RightMaster);
  }

<<<<<<< HEAD
  public void robotInit() {}

  @Override
  public void periodic() {}

  public void setSpeed(double rightMasterSpeed, double leftMasterSpeed) {

    m_RightMaster.set(ControlMode.Velocity, rightMasterSpeed);
    m_LeftMaster.set(ControlMode.Velocity, leftMasterSpeed);
  }

  public void highGoal(double setDistance) {}
}
=======
  private static double highSpeed = 0.2;
  private static double lowSpeed = 0.05;
  private static double intakeSpeed = -0;
  private static double trapSpeed = 0;
  private static double stopSpeed = 0;

  private double m_speed = 0;

  private CANSparkMax m_leftMaster = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_rightMaster = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_leftFollower = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_rightFollower = new CANSparkMax(0, MotorType.kBrushless);

  public Shooter() {
    m_leftMaster.restoreFactoryDefaults();
    m_rightMaster.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();

    m_leftMaster.setInverted(true);
    m_leftFollower.setInverted(true);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);

    m_rightMaster.burnFlash();
    m_leftMaster.burnFlash();
    m_leftFollower.burnFlash();
    m_rightFollower.burnFlash();
    
  }

  @Override
  public void periodic() {
        m_leftMaster.set(m_speed);
        m_rightMaster.set(m_speed);
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
        .andThen(new WaitCommand(3).andThen(this.stop()));
  }

  public Command stop() {
    return this.runOnce(() -> setShootingLevel(shootSpeed.STOP));
  }
}
>>>>>>> 053d3e4 (most likely working shooter, need to modify speed values)
