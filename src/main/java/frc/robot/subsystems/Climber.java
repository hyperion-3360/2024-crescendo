package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private TalonSRX m_climberRight = new TalonSRX(Constants.SubsystemConstants.kclimberRightId);
  private TalonSRX m_climberLeft = new TalonSRX(Constants.SubsystemConstants.kclimberLeftId);

  // declare 2 members, check fb for type and port, add port in constants
  public Climber() {

    m_climberLeft.setInverted(true);
    m_climberLeft.configFactoryDefault();
    m_climberRight.configFactoryDefault();
    m_climberLeft.follow(m_climberRight);
  }

  public void robotInit() {}

  @Override
  public void periodic() {}
}
