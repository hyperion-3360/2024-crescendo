package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum shoot {
HIGH, 
LOW
}

public class Shooter extends SubsystemBase {

    // Create new motors

    private CANSparkMax m_RightMaster = new CANSparkMax(Constants.ShooterConstants.kRightMasterid);
    private CANSparkMax m_RightFollower = new CANSparkMax(Constants.ShooterConstants.kRightFollowerid);
    private CANSparkMax m_LeftMaster = new CANSparkMax(Constants.ShooterConstants.kLeftMasterid);
    private CANSparkMax m_LeftFollower = new CANSparkMax(Constants.ShooterConstants.kLeftFollowerid);
    
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

public void setSpeed(double rightMasterSpeed, double leftMasterSpeed) {

    m_RightMaster.set(ControlMode.Velocity, rightMasterSpeed);
    m_LeftMaster.set(ControlMode.Velocity, leftMasterSpeed);

}
public void highGoal(double setDistance) {


}
}
