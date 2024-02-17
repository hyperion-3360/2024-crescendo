package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  // different states for the climber position
  public enum climberPos {
    TOP,
    STOP,
    INITAL,
    STALL
  }

  // instantiate the two neo 550 motors that are going to be used for the  climber
  private CANSparkMax m_climberLeft =
      new CANSparkMax(Constants.SubsystemConstants.kclimberLeftId, MotorType.kBrushless);
  private CANSparkMax m_climberRightMaster =
      new CANSparkMax(Constants.SubsystemConstants.kclimberRightId, MotorType.kBrushless);

  // private WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  // instantiating a relative encoder to detect the motors postion
  private RelativeEncoder m_encoder = m_climberRightMaster.getEncoder();

  // class variables to help control and set the climber
  private double m_speed;
  private double m_climberRampRate = 2; // was .2
  private double m_climberTarget = ClimberConstants.kTopTarget;
  private boolean isrunning = false;
  private double m_climberStallSpeed = 0.01;

  private double kP = 0.025;
  private double kI = 0.0001;
  private double kD = 0.0001;

  private PIDController m_PID = new PIDController(kP, kI, kD);

  // declare 2 members, check fb for type and port, add port in constants
  public Climber() {
    // configurating the motors and encoders
    m_climberLeft.restoreFactoryDefaults();
    m_climberRightMaster.restoreFactoryDefaults();

    m_climberLeft.follow(m_climberRightMaster, true);

    m_encoder.setPosition(0.0);

    // m_gyro.reset();

    m_climberRightMaster.setOpenLoopRampRate(m_climberRampRate);
    m_climberLeft.setOpenLoopRampRate(m_climberRampRate);

    m_climberRightMaster.setIdleMode(IdleMode.kCoast);
    m_climberLeft.setIdleMode(IdleMode.kCoast);

    m_climberRightMaster.burnFlash();
    m_climberLeft.burnFlash();
  }

  @Override
  public void periodic() {
    if (isrunning == true) {
      m_climberRightMaster.set(m_PID.calculate(m_encoder.getPosition(), m_climberTarget));
    }
    // safety measures to prevent the motors from burning on reenable
    if (DriverStation.isDisabled()) {
      climberGoToSelectedLevel(climberPos.TOP).cancel();
      climberGoToSelectedLevel(climberPos.INITAL).cancel();
      climberGoToSelectedLevel(climberPos.STALL).cancel();
    }
  }

  // private method to set the behavior for each state
  private void setClimberLevel(climberPos m_climberCheck) {
    switch (m_climberCheck) {
      case TOP:
        isrunning = true;
        m_climberTarget = Constants.ClimberConstants.kTopTarget;
        m_climberRightMaster.set(-m_speed);
        m_climberStallSpeed = 0;
        break;

      case STALL:
        isrunning = true;
        m_climberTarget = m_encoder.getPosition();
        m_PID.reset();
        break;

      case STOP:
        isrunning = false;
        m_climberRightMaster.stopMotor();
        break;

      case INITAL:
        isrunning = true;
        m_climberTarget = ClimberConstants.kstartPos;
        m_climberRightMaster.set(m_speed);
        m_climberStallSpeed = 0.0;
        break;
    }
  }

  // private void repositionement() {

  //     if (Math.abs(m_gyro.getRoll()) > 15){
  //              m_climberRightMaster.set(+ 0.1);
  //     }

  //     if (Math.abs(m_gyro.getRoll()) < -15){
  //              m_climberLeft.set(+ 0.1);
  //      }
  //          goToTarget();

  //     }

  // command to set the desired climber state
  public Command climberGoToSelectedLevel(climberPos m_climberCheck) {
    return this.runOnce(
        () -> {
          setClimberLevel(m_climberCheck);
        });
  }
}
