package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum shoot {
  HIGH,
  LOW
}

public class Shooter extends SubsystemBase {

  // Create new motors
  private CANSparkMax m_RightMaster =
      new CANSparkMax(Constants.SubsystemConstants.kShooterRightMasterId, MotorType.kBrushless);
  private CANSparkMax m_RightFollower =
      new CANSparkMax(Constants.SubsystemConstants.kShooterRightFollowerId, MotorType.kBrushless);
  private CANSparkMax m_LeftMaster =
      new CANSparkMax(Constants.SubsystemConstants.kShooterLeftMasterId, MotorType.kBrushless);
  private CANSparkMax m_LeftFollower =
      new CANSparkMax(Constants.SubsystemConstants.kShooterLeftFollowerId, MotorType.kBrushless);

  public void Shooter() {

    // Config motors

    m_LeftMaster.setInverted(true);
    m_LeftFollower.setInverted(true);

    m_RightMaster.restoreFactoryDefaults();
    m_RightFollower.restoreFactoryDefaults();
    m_LeftMaster.restoreFactoryDefaults();
    m_LeftFollower.restoreFactoryDefaults();

    m_LeftFollower.follow(m_LeftMaster);
    m_RightFollower.follow(m_RightMaster);
  }

  public void robotInit() {}

  @Override
  public void periodic() {}

  public void setSpeed(double rightMasterSpeed, double leftMasterSpeed) {
    m_RightMaster.set(rightMasterSpeed);
    m_LeftMaster.set(leftMasterSpeed);
  }

  public void highGoal(double setDistance) {}
}
