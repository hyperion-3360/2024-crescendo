package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.e_elevatorLevel;

public class Sequences {

  public static Command switchToIntakeMode(Elevator m_elevator) {
    return Commands.sequence(m_elevator.extendTheElevator(e_elevatorLevel.INTAKE));
  }

  public static Command switchToHigh(Elevator m_elevator) {
    return Commands.sequence(m_elevator.extendTheElevator(e_elevatorLevel.HIGH));
  }

  public static Command switchToLow(Elevator m_elevator) {
    return Commands.sequence(m_elevator.extendTheElevator(e_elevatorLevel.LOW));
  }
}
