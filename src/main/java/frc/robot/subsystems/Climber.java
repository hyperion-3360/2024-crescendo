package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

  public double triggerSpeed = 0.0;

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

    m_climberRightMaster.set(triggerSpeed);
    m_climberLeft.set(triggerSpeed);

    // System.out.println("pos " + m_encoder.getPosition() + " speed " + triggerSpeed);
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
