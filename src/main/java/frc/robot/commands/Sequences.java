package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;
import frc.robot.subsystems.Trap;

public class Sequences {
  // makes the climber sequence
  //   public static Command climberSequence(Climber m_climber, LEDs m_LED, Trap m_trap) {
  //     return Commands.sequence(
  //         Commands.runOnce(() -> m_LED.setState(State.CLIMBING)),
  //         m_climber
  //             .climberGoToSelectedLevel(climberPos.TOP)
  //             .andThen(new WaitCommand(2))
  //             .andThen(m_climber.climberGoToSelectedLevel(climberPos.INITAL)),
  //         new WaitUntilCommand(() -> m_climber.onClimberTarget())
  //             .andThen(
  //                 m_trap
  //                     .grabPosition()
  //                     .andThen(new WaitUntilCommand(0))
  //                     .andThen(m_trap.scoreNote())));
  //   }

  public static Command trapElevator(Elevator m_elevator, Trap m_trap) {
    return Commands.sequence(
        m_trap.prepareToClimb(),
        new WaitCommand(1.5),
        m_elevator.extendTheElevator(elevatorHeight.HIGH));
  }

  // sequence to feed the note to the trap and store it
  public static Command trapShoot(Shooter m_shooter, Trap m_trap) {
    return Commands.sequence(
        m_shooter.hookRelease(),
        m_shooter.setTargetLevel(levelSpeed.TRAP),
        m_shooter.setSpeedWithTarget(),
        new WaitUntilCommand(m_trap::trapHasNote),
        m_trap.storeNote(),
        new WaitCommand(2),
        m_shooter.stop());
  }
}
