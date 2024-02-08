package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.Conversions;

public class Climber extends SubsystemBase {

  public enum climberPos {
    TOP,
    BOTTOM,
    STOP,
    INITAL
  }

  private CANSparkMax m_climberLeft =
      new CANSparkMax(Constants.SubsystemConstants.kclimberLeftId, MotorType.kBrushless);
  private CANSparkMax m_climberRightMaster =
      new CANSparkMax(Constants.SubsystemConstants.kclimberRightId, MotorType.kBrushless);

  // private WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);

  private RelativeEncoder m_encoder = m_climberRightMaster.getEncoder();

  private double m_climberRampRate = 1; // was .2
  private double m_speed = 0.15;
  private double m_stallSpeed = 0.05;
  private double m_startPos = 0.75;
  private double m_climberTarget = m_startPos;

  // declare 2 members, check fb for type and port, add port in constants
  public Climber() {
    m_climberLeft.restoreFactoryDefaults();
    m_climberRightMaster.restoreFactoryDefaults();

    m_climberLeft.follow(m_climberRightMaster, true);

    m_encoder.setPosition(m_encoder.getPosition());

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

    System.out.println("enc pos in m " + encoderPositon() + " on target? " + onInitialTarget());
  }

  private void setClimberLevel(climberPos m_climberCheck) {
    switch (m_climberCheck) {
      case TOP:
        m_climberTarget = Constants.ClimberConstants.kTopTarget;
        m_climberRightMaster.set(m_speed);
        break;

      case BOTTOM:
        m_climberTarget = Constants.ClimberConstants.kBottomTarget;
        m_climberRightMaster.set(-m_speed);
        break;

      case STOP:
        m_climberRightMaster.stopMotor();
        break;

      case INITAL:
        m_climberTarget = m_startPos;
        m_climberRightMaster.set(-m_speed);
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

  private double encoderPositon() {
    double m_encoderPosition;
    m_encoderPosition = Conversions.NEOToMeters(9, 0.107526667, m_encoder.getPosition());
    return m_encoderPosition;
  }

  private boolean onTarget() {
    return Math.abs(m_climberTarget - encoderPositon())
        < Constants.ClimberConstants.kclimberDeadBand;
  }

  private boolean onInitialTarget() {
    return encoderPositon() >= m_climberTarget;
  }

  public Command climberGoToSelectedLevel(climberPos m_climberCheck) {
    return new SequentialCommandGroup(
        this.run(() -> setClimberLevel(m_climberCheck))
            .until(this::onTarget)
            .andThen(() -> setClimberLevel(climberPos.STOP)));
  }

  public Command setInitialPos() {
    return this.run(() -> m_climberRightMaster.set(m_speed))
        .until(this::onInitialTarget)
        .andThen(() -> m_climberRightMaster.stopMotor());
  }
}
