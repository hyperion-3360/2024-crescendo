package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;
import frc.robot.subsystems.Climber.e_climberCheck;
import frc.robot.subsystems.Elevator.e_elevatorLevel;
import frc.robot.subsystems.Shooter.shootSpeed;

public class Sequences {
  
   public static Command shootLow(
    Shooter m_shooter, Blocker m_blocker, Elevator m_elevator) {
    return Commands.sequence(
      m_blocker.setZero(),
      m_elevator.extendTheElevator(e_elevatorLevel.LOW).andThen
      (m_shooter.shoot(shootSpeed.LOW)).
      alongWith(new WaitCommand(1).andThen( m_blocker.letGoOfNote().
      andThen(new WaitCommand(1)))),
      m_shooter.stop(),
      m_blocker.setZero()
    );
  }

   public static Command shootHigh(
    Shooter m_shooter, Blocker m_blocker, Elevator m_elevator) {
    return Commands.sequence(
      m_blocker.setZero(),
      m_elevator.extendTheElevator(e_elevatorLevel.LOW).andThen(
      m_shooter.shoot(shootSpeed.HIGH)).
      alongWith(new WaitCommand(1).andThen( m_blocker.letGoOfNote().
      andThen(new WaitCommand(1)))),
      m_shooter.stop(),
      m_blocker.setZero()
    );
  }

   public static Command switchToIntakeMode(
    Shooter m_shooter, Blocker m_blocker, Elevator m_elevator) {
    return Commands.sequence(
      m_blocker.setZero(),
      m_elevator.extendTheElevator(e_elevatorLevel.LOW).andThen
      (m_shooter.shoot(shootSpeed.INTAKE))
    );
  }

   public static Command automaticTrapSequence(Trap m_trap, Shooter m_shooter) {
      return Commands.sequence(
        m_trap.grabPosition(),
        new WaitCommand(5),
        m_shooter.shoot(shootSpeed.TRAP),
        new WaitUntilCommand(() -> !m_trap.m_limitSwitch.get()),
        m_trap.scoreNote(),
        new WaitUntilCommand(() -> m_trap.m_limitSwitch.get()),
        m_trap.setZero()
      );
    }

    // public static Command manualTrapCommand(Trap m_trap) {
    //   return Commands.sequence(

    //   );
    // }

    // public static Command climberSequence(Climber m_climber) {
    //   return Command.sequence(
    //     m_climber.climberGoToSelectedLevel(e_climberCheck.BOTTOM),
    //     new WaitUntilCommand()
    //   );
    // }
}
