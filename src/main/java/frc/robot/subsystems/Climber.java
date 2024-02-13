package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
  private double m_climberRampRate = 2; // was .2
  private double m_speed = 0.2;
  private double m_climberTarget = ClimberConstants.kstartPos;
  private boolean isTop = true;
  private double m_climberStallSpeed = 0.01;

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

    m_climberRightMaster.setIdleMode(IdleMode.kBrake);
    m_climberLeft.setIdleMode(IdleMode.kBrake);

    m_climberRightMaster.burnFlash();
    m_climberLeft.burnFlash();

    climberGoToSelectedLevel(climberPos.INITAL);
  }

  @Override
  public void periodic() {
    // safety measures to prevent the motors from burning on reenable
    if (DriverStation.isDisabled()) {
      climberGoToSelectedLevel(climberPos.TOP).cancel();
      climberGoToSelectedLevel(climberPos.INITAL).cancel();
      climberGoToSelectedLevel(climberPos.STALL).cancel();
    }

    // // TODO I have no fucking clue if this actually works 👀
    // if (m_encoder.getPosition() != ClimberConstants.kTopTarget) {
    //   m_speed = m_climberStallSpeed;
    // }

    //   m_gyro.getRoll();

    //   repositionement();
  }

  // private method to set the behavior for each state
  private void setClimberLevel(climberPos m_climberCheck) {
    switch (m_climberCheck) {
      case TOP:
        isTop = true;
        m_climberTarget = Constants.ClimberConstants.kTopTarget;
        m_climberRightMaster.set(-0.1);
        m_climberStallSpeed = 0;
        break;

        // Quoicoubeh
      case STALL:
        m_climberRightMaster.set(m_climberStallSpeed);
        break;

      case STOP:
        m_climberRightMaster.stopMotor();
        break;

      case INITAL:
        isTop = false;
        m_climberTarget = ClimberConstants.kstartPos;
        m_climberRightMaster.set(m_speed);
        m_climberStallSpeed = 0.01;
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

  // boolean checking if the motors has reached its target
  public boolean onClimberTarget() {

    return m_encoder.getPosition() >= m_climberTarget;
  }

  // command to set the desired elevator state
  public Command climberGoToSelectedLevel(climberPos m_climberCheck) {
    return this.run(() -> setClimberLevel(m_climberCheck))
        .until(this::onClimberTarget)
        .andThen(() -> setClimberLevel(climberPos.STALL));
  }

  public Command climberManualControl(climberPos m_climberCheck) {
    return this.run(
        () -> {
          setClimberLevel(m_climberCheck);
          if (m_climberRightMaster.get() == m_climberStallSpeed) {
            m_climberTarget = m_encoder.getPosition();
          }
        });
  }
}
