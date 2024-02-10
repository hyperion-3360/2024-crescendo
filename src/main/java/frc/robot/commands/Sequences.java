package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.climberPos;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;

public class Sequences {

  // trap isn't finished
  // public static Command automaticTrapSequence(Trap m_trap, Shooter m_shooter) {
  //   return Commands.sequence(
  //     m_trap.grabPosition().andThen(m_shooter.shoot(shootSpeed.TRAP),
  //     (new WaitUntilCommand()))
  //   );
  // }

  // public static Command manualTrapCommand(Trap m_trap) {
  //   return Commands.sequence(

  //   );
  // }

  public static Command climberSequence(Climber m_climber) {
    return Commands.sequence(
        m_climber
            .climberGoToSelectedLevel(climberPos.TOP)
            .alongWith(new WaitCommand(2))
            .andThen(m_climber.climberGoToSelectedLevel(climberPos.INITAL)));
  }

  public static Command elevatorHigh(Elevator m_elevator, Shooter m_shooter) {
    return Commands.sequence(
        m_elevator.extendTheElevator(elevatorHeight.HIGH),
        m_shooter.setTargetLevel(levelSpeed.HIGH));
  }

  public static Command elevatorLow(Elevator m_elevator, Shooter m_shooter) {
    return Commands.sequence(
        m_elevator.extendTheElevator(elevatorHeight.LOW), m_shooter.setTargetLevel(levelSpeed.LOW));
  }
}
