package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trap extends SubsystemBase {

  private CANSparkMax motor1 =
      new CANSparkMax(Constants.TrapConstants.kmotor1ID, MotorType.kBrushed);

  public Trap() {}

  @Override
  public void periodic() {}
}
