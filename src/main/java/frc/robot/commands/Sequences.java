package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  public static Command elevatorHigh(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
            leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
            elevator.extendTheElevator(elevatorHeight.HIGH),
            new WaitCommand(1.5))
        .andThen(
            shooter
                .holdSpeed(levelSpeed.HIGH)
                .alongWith(
                    new WaitCommand(1).andThen(() -> leds.setState(State.SHOOT_READY_SPEAKER))));
  }

  public static Command elevatorFarHigh(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
            leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
            elevator.extendTheElevator(elevatorHeight.FAR_HIGH),
            new WaitCommand(1.5))
        .andThen(
            shooter
                .holdSpeed(levelSpeed.FAR_HIGH)
                .alongWith(
                    new WaitCommand(1.2).andThen(() -> leds.setState(State.SHOOT_READY_SPEAKER))));
  }

  public static Command elevatorLow(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
            leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
            elevator.extendTheElevator(elevatorHeight.LOW),
            new WaitCommand(1.5))
        .andThen(
            shooter
                .holdSpeed(levelSpeed.LOW)
                .alongWith(
                    new WaitCommand(0.7).andThen(() -> leds.setState(State.SHOOT_READY_SPEAKER))));
  }

  public static Command shoot(Shooter shooter, Elevator elevator, LEDs leds) {
    return Commands.sequence(
        shooter.hookRelease(),
        new WaitCommand(0.7),
        leds.runOnce(() -> leds.setState(State.SHOT_DONE)),
        shooter.stop(),
        elevator.extendTheElevator(elevatorHeight.INTAKE),
        shooter.hookIntake(),
        new WaitCommand(1.5),
        leds.runOnce(() -> leds.setState(State.IDLE)));
  }

  // intake sequence to set leds to the right state
  public static Command intakeSequence(Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
            Commands.runOnce(() -> m_LED.setState(State.INTAKE_ROLLING)),
            m_shooter
                .intake()
                .andThen(() -> m_LED.setState(State.NOTE_INSIDE))
                .andThen(new WaitCommand(2).andThen(() -> m_LED.setState(State.IDLE))))
        .handleInterrupt(() -> m_LED.setState(State.IDLE));
  }

  // sequence to feed the note to the trap and store it
  public static Command trapShoot(Shooter m_shooter, Trap m_trap) {
    return Commands.sequence(
        m_shooter.hookRelease(),
        m_shooter.setTargetLevel(levelSpeed.TRAP),
        m_shooter.setSpeedWithTarget(),
        m_trap.grabPosition(),
        new WaitUntilCommand(m_trap::trapHasNote),
        new PrintCommand("limit switch on"),
        m_trap.storeNote(),
        new WaitCommand(2),
        m_shooter.stop());
  }

  // sequence to score note in trap
  // public static Command trapScore(Trap m_trap) {
  //   return Commands.sequence(m_trap.scoreNote(), new WaitCommand(2), m_trap.dunkNote());
  // }

  // sequence lift elevator and start wheels to climb !! wait will have to be modified !!
  public static Command climbElevator(Elevator elevator, Shooter shooter, Trap trap) {
    return Commands.sequence(
        trap.prepareToClimb(),
        new WaitCommand(0.5),
        elevator.extendTheElevator(elevatorHeight.HIGH),
        new WaitCommand(1),
        shooter.holdSpeed(levelSpeed.CLIMB),
        new WaitCommand(5),
        shooter.stop());
  }
}
