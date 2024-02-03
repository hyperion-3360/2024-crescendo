package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.CurveFunction;

public class Climber extends SubsystemBase {

    public enum e_climberCheck{
        TOP,
        BOTTOM,
        GRIP_MODE,
        STOP
    }

    private CANSparkMax m_climberLeft = new CANSparkMax(Constants.SubsystemConstants.kclimberLeftId, MotorType.kBrushless);
    private CANSparkMax  m_climberRightMaster = new CANSparkMax(Constants.SubsystemConstants.kclimberRightId, MotorType.kBrushless);

    private WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);

    private RelativeEncoder m_encoder = m_climberRightMaster.getEncoder();

    private double m_climberTarget = 0.0;

    private CurveFunction m_curve = new CurveFunction();

    // declare 2 members, check fb for type and port, add port in constants
    public  Climber() {
        m_climberLeft.restoreFactoryDefaults();
        m_climberRightMaster.restoreFactoryDefaults();

        m_climberRightMaster.setInverted(true);

        m_climberLeft.follow(m_climberRightMaster, true);

        m_encoder.setPosition(0.0);

        m_gyro.reset();
    }
    
    public void robotInit()
    {

    }
    
    @Override
    public void periodic()
    {
        if (this.onTarget()) {
            this.stop();
          } else {
            Double adjustedSpeed = m_curve.adjustPeriodic();
            if (adjustedSpeed != null) {
              m_climberRightMaster.set(adjustedSpeed);
      
            }
          }

          m_gyro.getRoll();

          repositionement();

          System.out.println("ENCODER POSITION " + encoderPositon());
    }

    public void setClimberSpeed(double m_climberSpeed) {
       m_climberSpeed = m_curve.getMotorSpeed(m_climberSpeed);
       m_climberRightMaster.set(m_climberSpeed);
    }

    private void setClimberLevel(e_climberCheck m_climberCheck) {
        switch (m_climberCheck) {
                case TOP:
                m_climberTarget = Constants.ClimberConstants.kTopTarget;
                break;
        
                case BOTTOM:
                m_climberTarget = Constants.ClimberConstants.kBottomTarget;
                break;
                
                case GRIP_MODE:
                m_climberTarget = Constants.ClimberConstants.kGripTarget;
                break;

                case STOP:
                setClimberSpeed(0.0);
        }
    }

    private void repositionement() {

        if (Math.abs(m_gyro.getRoll()) > 15){
                 m_climberRightMaster.set(+ 0.1);
        }

        if (Math.abs(m_gyro.getRoll()) < -15){
                 m_climberLeft.set(+ 0.1);
         }
             goToTarget();
               
        }

    private void stop() {
        m_climberRightMaster.stopMotor();
    }

    private double encoderPositon() {
        return m_encoder.getPosition() / 360;
    }

    private boolean negativeClimberChecker() {
        if (m_climberTarget > m_encoder.getPosition()) {
            return true;
        }
        return false;
    }

    private boolean onTarget() {
        if (negativeClimberChecker() == true){
        return Math.abs(m_climberTarget - encoderPositon()) > 
        Constants.ClimberConstants.kclimberDeadBand;
    }
        return Math.abs(m_climberTarget + encoderPositon()) >
         Constants.ClimberConstants.kclimberDeadBand;
   }

   private void goToTarget() {
    if (negativeClimberChecker() == true) {
        setClimberSpeed(-0.20);
    }
    setClimberSpeed(0.20);
   }

    public Command climberGoToSelectedLevel(e_climberCheck m_climberCheck) {
        return new SequentialCommandGroup(
            this.runOnce(
                () -> {
                    setClimberLevel(m_climberCheck);
                }
            ),
            run(() -> goToTarget()).until(this::onTarget).
            andThen(() -> setClimberLevel(e_climberCheck.STOP))
            );
    }


}