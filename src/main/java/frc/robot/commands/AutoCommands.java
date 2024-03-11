package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;

public class AutoCommands {
  public static Command autoShoot(Elevator elevator, Shooter shooter) {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.HIGH),
        shooter.setTargetLevel(levelSpeed.HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1.15),
        shooter.hookRelease(),
        new WaitCommand(0.5),
        shooter.stop(),
        elevator.extendTheElevator(elevatorHeight.INTAKE));
  }

  public static Command autoFarShoot1(Elevator elevator, Shooter shooter) {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.AUTOFAR1),
        shooter.setTargetLevel(levelSpeed.FAR_HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1.15),
        shooter.hookRelease(),
        new WaitCommand(0.5),
        shooter.stop(),
        elevator.extendTheElevator(elevatorHeight.INTAKE));
  }

  public static Command autoFarShoot2(Elevator elevator, Shooter shooter) {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.AUTOFAR2),
        shooter.setTargetLevel(levelSpeed.FAR_HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1.15),
        shooter.hookRelease(),
        new WaitCommand(0.5),
        shooter.stop(),
        elevator.extendTheElevator(elevatorHeight.INTAKE));
  }

  public static Command autoFarShoot3(Elevator elevator, Shooter shooter) {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.AUTOFAR3),
        shooter.setTargetLevel(levelSpeed.FAR_HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1.15),
        shooter.hookRelease(),
        new WaitCommand(0.5),
        shooter.stop(),
        elevator.extendTheElevator(elevatorHeight.INTAKE));
  }

  public static Command autoFarShoot4(Elevator elevator, Shooter shooter) {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.AUTOFAR4),
        shooter.setTargetLevel(levelSpeed.FAR_HIGH),
        shooter.setSpeedWithTarget(),
        new WaitCommand(1),
        shooter.hookRelease().withTimeout(0.3),
        new WaitCommand(0.5),
        shooter.stop(),
        elevator.extendTheElevator(elevatorHeight.INTAKE));
  }
}
