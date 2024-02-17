package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;

public class Sequences {
  // TODO START OF NEW SEQUENCES FOR ELEVATOR AND SHOOTING

  public static Command elevatorHigh(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
        leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
        elevator.extendTheElevator(elevatorHeight.HIGH),
        new WaitCommand(1.5),
        shooter.setTargetLevel(levelSpeed.HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1),
        leds.runOnce(() -> leds.setState(State.SHOOT_READY_SPEAKER)));
  }

  public static Command elevatorFarHigh(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
        leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
        elevator.extendTheElevator(elevatorHeight.FAR_HIGH),
        new WaitCommand(1.5),
        shooter.setTargetLevel(levelSpeed.FAR_HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1),
        leds.runOnce(() -> leds.setState(State.SHOOT_READY_SPEAKER)));
  }

  public static Command elevatorLow(Elevator elevator, Shooter shooter, LEDs leds) {
    return Commands.sequence(
            leds.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
            elevator.extendTheElevator(elevatorHeight.LOW),
            new WaitCommand(1.5),
            shooter.setTargetLevel(levelSpeed.LOW))
        .andThen(shooter.run(() -> shooter.setSpeedWithTarget()));
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
}
