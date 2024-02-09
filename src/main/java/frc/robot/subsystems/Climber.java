package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  public enum climberPos {
    TOP,
    STOP,
    INITAL,
    STALL
  }

  private CANSparkMax m_climberLeft =
      new CANSparkMax(Constants.SubsystemConstants.kclimberLeftId, MotorType.kBrushless);
  private CANSparkMax m_climberRightMaster =
      new CANSparkMax(Constants.SubsystemConstants.kclimberRightId, MotorType.kBrushless);

  // private WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);

  private RelativeEncoder m_encoder = m_climberRightMaster.getEncoder();

  private double m_climberRampRate = 1; // was .2
  private double m_speed = 0.2;
  private double m_climberTarget = ClimberConstants.kstartPos;
  private boolean isTop = false;
  private double m_climberStallSpeed;

  // declare 2 members, check fb for type and port, add port in constants
  public Climber() {
    m_climberLeft.restoreFactoryDefaults();
    m_climberRightMaster.restoreFactoryDefaults();

    m_climberLeft.follow(m_climberRightMaster, true);

    m_encoder.setPosition(0.0);

    // m_gyro.reset();

    m_climberRightMaster.setOpenLoopRampRate(m_climberRampRate);
    m_climberLeft.setOpenLoopRampRate(m_climberRampRate);

    m_climberRightMaster.burnFlash();
    m_climberLeft.burnFlash();

    // setInitialPos();
  }

  @Override
  public void periodic() {

    // if (DriverStation.isDisabled()) {
    //   m_climberTarget = m_encoder.getPosition();
    //   m_climberRightMaster.set(0.0);
    // }

    //   m_gyro.getRoll();

    //   repositionement();

    System.out.println("enc pos " + m_encoder.getPosition() + " on target? " + onTarget());
  }

  private void setClimberLevel(climberPos m_climberCheck) {
    switch (m_climberCheck) {
      case TOP:
        isTop = true;
        m_climberTarget = Constants.ClimberConstants.kTopTarget;
        m_climberRightMaster.set(-m_speed);
        m_climberStallSpeed = 0;
        break;

      case STALL:
        m_climberRightMaster.set(m_climberStallSpeed);
        break;

      case STOP:
        m_climberRightMaster.stopMotor();
        break;

      case INITAL:
        isTop = false;
        m_climberTarget = ClimberConstants.kstartPos;
        m_climberRightMaster.set(0.1);
        m_climberStallSpeed = 0.03;
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
  private boolean onTarget() {
    if (isTop == true) {
      return m_encoder.getPosition() <= m_climberTarget;
    } else {
      return m_encoder.getPosition() >= m_climberTarget;
    }
  }

  public Command climberGoToSelectedLevel(climberPos m_climberCheck) {
    return this.run(() -> setClimberLevel(m_climberCheck))
        .until(this::onTarget)
        .andThen(() -> setClimberLevel(climberPos.STALL));
  }
}
