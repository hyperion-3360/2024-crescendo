package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
public void robotInit(){

}

@Override
public void periodic() {
        
}
}
