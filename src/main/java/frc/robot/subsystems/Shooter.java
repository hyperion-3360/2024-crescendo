package frc.robot.subsystems;



import java.util.logging.Level;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public enum e_shoot {
HIGH, 
LOW
}
public class Shooter extends SubsystemBase {

    // Create new motors
    private CANSparkMax m_rightMaster = new CANSparkMax(Constants.SubsystemConstants.kShooterRightMasterId, MotorType.kBrushless);
    private CANSparkMax m_rightFollower = new CANSparkMax(Constants.SubsystemConstants.kShooterRightFollowerId, MotorType.kBrushless);
    private CANSparkMax m_leftMaster = new CANSparkMax(Constants.SubsystemConstants.kShooterLeftMasterId, MotorType.kBrushless);
    private CANSparkMax m_leftFollower = new CANSparkMax(Constants.SubsystemConstants.kShooterLeftFollowerId, MotorType.kBrushless);
    
    public Shooter() {

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

    public Command setSpeed(double rightMasterSpeed, double leftMasterSpeed) {
        return this.runOnce(() -> {
        m_rightMaster.set(rightMasterSpeed);
        m_leftMaster.set(leftMasterSpeed);
        });
    }

    public Command Stop() {
    return this.runOnce(() -> {
        m_leftMaster.stopMotor();
        m_rightMaster.stopMotor();
     });
    }
    
    


    
    private void setShootingLevel(e_shoot shoot) {
        
    switch(shoot) {

    case LOW:
    m_leftMaster.set(0.05);
    m_rightMaster.set(0.05);
    break;

    case HIGH:
    m_leftMaster.set(0.8);
    m_rightMaster.set(0.8);
    break;
     }
    }

    public Command shooting(e_shoot shoot) {
        return new SequentialCommandGroup(
        this.runOnce(
            () -> {
                setShootingLevel(shoot);
            }).andThen(setSpeed(0, 0))
            //run(() -> setShootingLevel(shoot)).unless(null) //TODO add elevator not at position condition
        );
    }

}

