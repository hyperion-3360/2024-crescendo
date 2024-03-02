package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.State;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;
import frc.robot.subsystems.Trap;

public class Sequences {

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

  public static Command elevatorFarHighFromClimb(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
            leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
            elevator.extendTheElevator(elevatorHeight.FAR_HIGH_CLIMB),
            new WaitCommand(1.5))
        .andThen(
            shooter
                .holdSpeed(levelSpeed.FAR_HIGH)
                .alongWith(
                    new WaitCommand(1.2).andThen(() -> leds.setState(State.SHOOT_READY_SPEAKER))));
  }

  public static Command elevatorFarHighFromAmp(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
            leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
            elevator.extendTheElevator(elevatorHeight.FAR_HIGH_AMP),
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
            leds.runOnce(() -> leds.setState(State.IDLE)))
        .withTimeout(4)
        .handleInterrupt(
            () -> elevator.extendTheElevator(elevatorHeight.INTAKE).andThen(shooter.hookIntake()))
        .andThen(() -> leds.setState(State.IDLE));
  }

  // intake sequence to set leds to the right state
  public static Command intakeSequence(
      Shooter shooter, LEDs leds, CommandXboxController controller) {
    return Commands.sequence(
            Commands.runOnce(() -> leds.setState(State.INTAKE_ROLLING)),
            intakeRumbleOn(controller, shooter).alongWith(shooter.intake()),
            leds.runOnce(() -> leds.setState(State.NOTE_INSIDE))
                .andThen(new WaitCommand(2).andThen(() -> leds.setState(State.IDLE))))
        .handleInterrupt(
            () -> {
              leds.setState(State.IDLE);
            });
  }

  // sequence to feed the note to the trap and store it
  public static Command trapShoot(Shooter m_shooter, Trap m_trap) {
    return Commands.sequence(
        m_trap.grabPosition(),
        new WaitCommand(0.5),
        m_shooter.hookRelease(),
        m_shooter.setTargetLevel(levelSpeed.TRAP),
        m_shooter.setSpeedWithTarget(),
        new WaitUntilCommand(m_trap::trapHasNote),
        m_trap.closeFinger(),
        m_trap.storeNote(),
        new WaitCommand(2),
        m_shooter.stop());
  }

  // sequence to score note in trap
  public static Command trapScore(Trap m_trap) {
    return Commands.sequence(
        m_trap.dunkNote(),
        new WaitCommand(1),
        m_trap.prepareToDisable1(),
        new WaitCommand(0.2),
        m_trap.prepareToDisable2());
  }

  // sequence lift elevator and start wheels to climb !! wait will have to be modified !!
  // climb with the arm
  public static Command climbElevatorNote(Elevator elevator, Shooter shooter, Trap trap) {
    return Commands.sequence(
        trap.prepareToClimb(),
        new WaitCommand(0.5),
        elevator.extendTheElevator(elevatorHeight.HIGH),
        new WaitCommand(1),
        shooter.holdSpeed(levelSpeed.CLIMB),
        new WaitCommand(5),
        shooter.stop());
  }

  // climb without the arm
  public static Command climbElevator(Elevator elevator, Shooter shooter) {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.HIGH),
        new WaitCommand(1),
        shooter.holdSpeed(levelSpeed.CLIMB),
        new WaitCommand(5),
        shooter.stop());
  }

  public static Command blockShooterGears(
      Shooter shooter, LEDs leds, CommandXboxController controller) {
    return Commands.sequence(
        shooter.stop(),
        shooter.gearBlockMode(),
        Commands.parallel(
            climbRumble(controller), (leds.runOnce(() -> leds.setState(State.GEAR_BLOCKED)))));
  }

  public static Command rumble(CommandXboxController controller, boolean on) {
    if (on) {
      return Commands.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.2));
    } else {
      return Commands.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
    }
  }

  public static Command intakeRumbleOn(CommandXboxController controller, Shooter shooter) {
    return Commands.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.6))
        .until(shooter::hasNote)
        .andThen(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  public static Command climbRumble(CommandXboxController controller) {
    return Commands.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.2))
        .withTimeout(5)
        .andThen(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0));
  }
}
