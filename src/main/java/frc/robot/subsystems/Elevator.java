package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonSRX m_elevatorRight = new TalonSRX(Constants.SubsystemConstants.kelevatorRightId);
  private TalonSRX m_elevatorLeft = new TalonSRX(Constants.SubsystemConstants.kelevatorLeftId);

  // same as climber.java
  public void Elevator() {

    m_elevatorLeft.setInverted(true);
    m_elevatorLeft.configFactoryDefault();
    m_elevatorRight.configFactoryDefault();
    m_elevatorLeft.follow(m_elevatorRight);
  }

  public void robotInit() {}

  @Override
  public void periodic() {}
}
