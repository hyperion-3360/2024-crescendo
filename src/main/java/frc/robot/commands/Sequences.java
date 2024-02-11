package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.climberPos;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.State;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;
import frc.robot.subsystems.Trap;

public class Sequences {
  // makes the climber sequence
  public static Command climberSequence(Climber m_climber, LEDs m_LED, Trap m_trap) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.CLIMBING)),
        m_climber
            .climberGoToSelectedLevel(climberPos.TOP)
            .andThen(new WaitCommand(2))
            .andThen(m_climber.climberGoToSelectedLevel(climberPos.INITAL)),
        // Commands.runOnce(() -> m_LED.setState(State.)),
        m_trap.grabPosition().andThen(m_trap.scoreNote())
        // .andThen(() -> m_LED.setState(State.)
        );
  }

  // the sequence to set the elevator to high
  public static Command elevatorHigh(Elevator m_elevator, Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.PREPARE_SHOT)),
        m_elevator
            .extendTheElevator(elevatorHeight.HIGH)
            .andThen(() -> m_LED.setState(State.SHOOT_READY)),
        m_shooter.setTargetLevel(levelSpeed.HIGH));
  }

  // the sequence to set the elevator to low
  public static Command elevatorLow(Elevator m_elevator, Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.PREPARE_SHOT)),
        m_elevator
            .extendTheElevator(elevatorHeight.LOW)
            .andThen(() -> m_LED.setState(State.SHOOT_READY)),
        m_shooter.setTargetLevel(levelSpeed.LOW));
  }

  // the sequence to make the shooter shoot to the desired level
  public static Command shoot(Shooter m_shooter, Elevator m_elevator, LEDs m_LED) {
    return Commands.sequence(
        m_shooter.shootTo().andThen(() -> m_LED.setState(State.SHOT_DONE)),
        m_elevator
            .extendTheElevator(elevatorHeight.INTAKE)
            .andThen(() -> m_LED.setState(State.IDLE)));
  }

  public static Command intakeSequence(Shooter m_shooter, LEDs m_LED) {
    return Commands.sequence(
        Commands.runOnce(() -> m_LED.setState(State.INTAKE_ROLLING)),
        m_shooter.intake().andThen(() -> m_LED.setState(State.NOTE_INSIDE)));
  }

  public static Command trapShoot(Shooter m_shooter, Trap m_trap) {
    return Commands.sequence(
        m_shooter.hookRelease(),
        m_shooter.setTargetLevel(levelSpeed.TRAP),
        m_shooter.setSpeedWithTarget(),
        new WaitUntilCommand(m_trap::trapHasNote),
        m_shooter.stop());
  }
}
