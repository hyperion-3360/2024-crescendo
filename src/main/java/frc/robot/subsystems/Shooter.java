package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase {
  public enum shootSpeed {
    HIGH,
    LOW,
    INTAKE,
    TRAP,
    STOP
  }

  private static double highSpeed = 0.05;
  private static double lowSpeed = 0.05;
  private static double intakeSpeed = -0;
  private static double trapSpeed = 0;
  private static double stopSpeed = 0;

  private double m_speed = 0;

  private CANSparkMax m_leftMaster = new CANSparkMax(9, MotorType.kBrushless);

  public Shooter() {
    m_leftMaster.restoreFactoryDefaults();
    m_leftMaster.setInverted(true);
    m_leftMaster.burnFlash();
  }

  @Override
  public void periodic() {
    m_leftMaster.set(0.05);
  }

  private void setShootingLevel(shootSpeed shoot) {

    switch (shoot) {
      case LOW:
        m_speed = lowSpeed;
        break;

      case HIGH:
        m_speed = highSpeed;
        break;

      case STOP:
        m_speed = stopSpeed;
        break;

      case TRAP:
        m_speed = trapSpeed;
        break;

      case INTAKE:
        m_speed = intakeSpeed;
        break;
    }
  }

  public Command shoot(shootSpeed shootSpeed) {
    return this.runOnce(() -> setShootingLevel(shootSpeed))
        .andThen(new WaitCommand(5).andThen(() -> setShootingLevel(shootSpeed.STOP)));
  }
}
