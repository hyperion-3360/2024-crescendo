package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
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

  // instantiating a relative encoder to detect the motors postion
  private RelativeEncoder m_encoder = m_climberRightMaster.getEncoder();

  // class variables to help control and set the climber
  private double m_climberRampRate = 1; // was .2
  private double m_climberTarget = ClimberConstants.kTopTarget;
  private boolean isrunning = false;

  public double triggerSpeed = 0.0;

  private double kP = 0.025;
  private double kI = 0.0007; // was 0.0001
  private double kD = 0.0004; // was 0.0001

  private double ks = 0.001;
  private double kg = 1.335; // 1.35 = Volt
  private double kv = 1.5; // 1.5 = Volt * second / meters
  private double ka = 2; // 2 = Volt *second^2 / meters

  private ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ks, kg, kv, ka);

  private PIDController m_PID = new PIDController(kP, kI, kD);

  // declare 2 members, check fb for type and port, add port in constants
  public Climber() {
    // configurating the motors and encoders
    m_climberLeft.restoreFactoryDefaults();
    m_climberRightMaster.restoreFactoryDefaults();

    m_climberLeft.follow(m_climberRightMaster, true);

    m_encoder.setPosition(0.0);

    m_climberRightMaster.setOpenLoopRampRate(m_climberRampRate);
    m_climberLeft.setOpenLoopRampRate(m_climberRampRate);

    m_climberRightMaster.setIdleMode(IdleMode.kBrake);
    m_climberLeft.setIdleMode(IdleMode.kBrake);

    m_climberRightMaster.burnFlash();
    m_climberLeft.burnFlash();
  }

  @Override
  public void periodic() {
    // if (isrunning == true) {
    //   m_climberRightMaster.set(
    //       m_PID.calculate(m_encoder.getPosition(), m_climberTarget)
    //           + m_feedforward.calculate(-0.9));
    // }

    m_climberRightMaster.set(triggerSpeed);
    m_climberLeft.set(triggerSpeed);
    // safety measures to prevent the motors from burning on reenable
    if (DriverStation.isDisabled()) {
      climberGoToSelectedLevel(climberPos.TOP).cancel();
      climberGoToSelectedLevel(climberPos.INITAL).cancel();
      climberGoToSelectedLevel(climberPos.STALL).cancel();
      climberGoToSelectedLevel(climberPos.STOP);
      m_PID.reset();
    }

    System.out.println("pos " + m_encoder.getPosition() + " speed " + triggerSpeed);
  }

  // private method to set the behavior for each state
  private void setClimberLevel(climberPos m_climberCheck) {
    switch (m_climberCheck) {
      case TOP:
        isrunning = true;
        m_climberTarget = Constants.ClimberConstants.kTopTarget;
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
        break;
    }
  }

  // command to set the desired climber state
  public Command climberGoToSelectedLevel(climberPos m_climberCheck) {
    return this.runOnce(
        () -> {
          setClimberLevel(m_climberCheck);
        });
  }

  public Command setSpeed1() {
    return this.run(() -> triggerSpeed = Math.pow(new XboxController(1).getLeftTriggerAxis(), 3));
  }

  public Command setSpeed2() {
    return this.run(() -> triggerSpeed = -Math.pow(new XboxController(1).getRightTriggerAxis(), 3));
  }

  public Command stop() {
    return this.run(() -> triggerSpeed = 0.0);
  }
}
