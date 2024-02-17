package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.State;
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

  // the sequence to set the elevator to high and change led, as well as set shooter target level
  public static Command elevatorHigh(Elevator m_elevator, Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.PREPARE_SHOT_SPEAKER)),
        m_elevator
            .extendTheElevator(elevatorHeight.HIGH)
            .andThen(new WaitCommand(1.5))
            // .andThen(new WaitUntilCommand(() -> m_elevator.onTarget()))
            .andThen(() -> m_LED.setState(State.SHOOT_READY_SPEAKER)),
        m_shooter.setTargetLevel(levelSpeed.HIGH));
  }

  public static Command trapElevator(Elevator m_elevator, Trap m_trap) {
    return Commands.sequence(
        m_trap.prepareToClimb(),
        // new WaitCommand(0.5),
        m_elevator.extendTheElevator(elevatorHeight.HIGH));
  }

  // the sequence to set the elevator to low and change led, as well as set shooter target level
  public static Command elevatorLow(Elevator m_elevator, Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.PREPARE_SHOT_AMP)),
        m_elevator
            .extendTheElevator(elevatorHeight.LOW)
            .andThen(new WaitCommand(1.5))
            .andThen(() -> m_LED.setState(State.SHOOT_READY_AMP)),
        m_shooter.setTargetLevel(levelSpeed.LOW));
  }

  // the sequence to make the shooter shoot to the desired level and change leds
  public static Command shoot(Shooter m_shooter, Elevator m_elevator, LEDs m_LED) {
    return Commands.sequence(
        m_shooter.shootTo().andThen(() -> m_LED.setState(State.SHOT_DONE)),
        m_elevator
            .extendTheElevator(elevatorHeight.INTAKE)
            .andThen(new WaitCommand(2))
            .andThen(() -> m_LED.setState(State.IDLE)));
  }

  // intake sequence to set leds to the right state
  public static Command intakeSequence(Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.INTAKE_ROLLING)),
        m_shooter
            .intake()
            .andThen(() -> m_LED.setState(State.NOTE_INSIDE))
            .andThen(new WaitCommand(2).andThen(() -> m_LED.setState(State.IDLE))));
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
