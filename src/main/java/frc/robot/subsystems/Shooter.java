package frc.robot.subsystems;



import java.util.logging.Level;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum shoot {
HIGH, 
LOW
}

public class Shooter extends SubsystemBase {

    // Create new motors

    private static final double stopMotorShooter = 0;
    private CANSparkMax m_rightMaster = new CANSparkMax(Constants.SubsystemConstants.kShooterRightMasterId, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(Constants.SubsystemConstants.kShooterRightFollowerId, MotorType.kBrushless);
    private CANSparkMax m_leftMaster = new CANSparkMax(Constants.SubsystemConstants.kShooterLeftMasterId, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(Constants.SubsystemConstants.kShooterLeftFollowerId, MotorType.kBrushless);
    
    public void Shooter() {

        // Config motors
        m_leftMaster.setInverted(true);
        m_leftFollower.setInverted(true);

        m_rightMaster.restoreFactoryDefaults();
        m_rightFollower.restoreFactoryDefaults();
        m_leftMaster.restoreFactoryDefaults();
        m_leftFollower.restoreFactoryDefaults();

        m_leftFollower.follow(m_leftMaster);
        m_rightFollower.follow(m_rightMaster);
        
    }

    public void robotInit(){


    }

    @Override
    public void periodic() {
            
    }

    public void setSpeed(double rightMasterSpeed, double leftMasterSpeed) {
        m_rightMaster.set(rightMasterSpeed);
        m_leftMaster.set(leftMasterSpeed);

    }

    public void highGoal(double setDistance) {


    }

    public Command Stop() {
        
        if() {

         m_leftMaster.set(stopMotorShooter);
         m_rightMaster.set(stopMotorShooter);


    } else {

        return null;


    
    }
    }


    
    public void level(shoot) {
        

    switch(shoot) {

    case LOW:

    m_leftMaster.set(0.3);
    m_rightMaster.set(0.3);

    break;

    case HIGH:

    m_leftMaster.set(0.8);
    m_rightMaster.set(0.8);


    }


    }

}

