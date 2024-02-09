package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.climberPos;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.e_elevatorLevel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shootSpeed;

public class Sequences {

  public static Command shootLow(Shooter m_shooter, Blocker m_blocker, Elevator m_elevator) {
    return Commands.sequence(
        m_blocker.hookIntake(),
        m_shooter
            .shoot(shootSpeed.LOW)
            .andThen(
                new WaitCommand(1).andThen(m_blocker.hookRelease().alongWith(new WaitCommand(2)))),
        m_shooter.stop(),
        m_blocker.hookIntake(),
        m_elevator.extendTheElevator(e_elevatorLevel.INTAKE));
  }

  public static Command shootHigh(Shooter m_shooter, Blocker m_blocker, Elevator m_elevator) {
    return Commands.sequence(
        m_blocker.hookIntake(),
        m_shooter
            .shoot(shootSpeed.HIGH)
            .andThen(
                new WaitCommand(1).andThen(m_blocker.hookRelease().alongWith(new WaitCommand(2)))),
        m_shooter.stop(),
        m_blocker.hookIntake(),
        m_elevator.extendTheElevator(e_elevatorLevel.INTAKE));
  }

  public static Command takeNote(Shooter m_shooter, Elevator m_elevator, Blocker m_blocker) {
    return Commands.sequence(
        m_blocker.hookIntake(),
        m_shooter.shoot(shootSpeed.INTAKE),
        m_shooter.stop(),
        m_blocker.hookIntake());
  }

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
}
