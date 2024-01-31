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
import frc.robot.math.CurveFunction;

public class Shooter extends SubsystemBase {
    public enum e_shoot {
        HIGH, 
        LOW
    }
    
    // Create new motors
   // private CANSparkMax m_rightMaster = new CANSparkMax(Constants.SubsystemConstants.kShooterRightMasterId, MotorType.kBrushless);
   // private CANSparkMax m_rightFollower = new CANSparkMax(Constants.SubsystemConstants.kShooterRightFollowerId, MotorType.kBrushless);
    private CANSparkMax m_leftMaster = new CANSparkMax(Constants.SubsystemConstants.kShooterLeftMasterId, MotorType.kBrushless);
   // private CANSparkMax m_leftFollower = new CANSparkMax(Constants.SubsystemConstants.kShooterLeftFollowerId, MotorType.kBrushless);
    
    private CurveFunction m_curve = new CurveFunction();

    private Elevator m_elevator = new Elevator();

    public Shooter() {

        // Config motors
        //m_leftMaster.setInverted(true);
        m_leftMaster.setInverted(true);

       // m_rightMaster.restoreFactoryDefaults();
       // m_rightFollower.restoreFactoryDefaults();
        m_leftMaster.restoreFactoryDefaults();
      //  m_leftFollower.restoreFactoryDefaults();

        // m_leftFollower.follow(m_leftMaster);
        // m_rightFollower.follow(m_rightMaster);
        
    }

    public void robotInit(){


    }

    @Override
    public void periodic() {
            
    }

    public Command setSpeed(double m_rightMasterSpeed, double m_leftMasterSpeed) {
        return this.runOnce(() -> {
       // m_rightMaster.set(rightMasterSpeed);
        m_leftMaster.set(m_leftMasterSpeed);
        });
    }

    public Command Stop() {
    return this.runOnce(() -> {
        m_leftMaster.stopMotor();
      //  m_rightMaster.stopMotor();
     });
    }
    
    


    
    private void setShootingLevel(e_shoot shoot) {
        
    switch(shoot) {

    case LOW:
    m_leftMaster.set(0.05);
   // m_rightMaster.set(0.05);
    break;

    case HIGH:
    m_leftMaster.set(0.08);
   // m_rightMaster.set(0.8);
    break;
     }
    }

    public Command shooting(e_shoot shoot) {
        return new SequentialCommandGroup(
        this.runOnce(
            () -> {
                setShootingLevel(shoot);
            }),
            run(() -> setShootingLevel(shoot))
            //.unless(() -> m_elevator.onTarget() == false)
            .andThen(setSpeed(0, 0))
        );
    }

}

