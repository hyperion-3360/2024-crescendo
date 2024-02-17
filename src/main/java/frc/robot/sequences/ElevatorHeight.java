package frc.robot.sequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.State;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.levelSpeed;

public class ElevatorHeight {

  private static Elevator elevator;
  private static LEDs leds;
  private static Shooter shooter;

  public ElevatorHeight() {
    elevatorHigh().addRequirements(elevator, leds);
    elevatorLow().addRequirements(elevator, leds);
    elevatorIntake().addRequirements(elevator, leds);
  }

  public static Command elevatorHigh() {
    return Commands.sequence(
        Commands.runOnce(() -> leds.setState(State.PREPARE_SHOT_SPEAKER)),
        elevator
            .extendTheElevator(elevatorHeight.HIGH)
            .andThen(new WaitCommand(1.5))
            // .andThen(new WaitUntilCommand(() -> m_elevator.onTarget()))
            .andThen(() -> leds.setState(State.SHOOT_READY_SPEAKER)),
        shooter.setTargetLevel(levelSpeed.HIGH));
  }

  public static Command elevatorLow() {
    return Commands.sequence(
        Commands.runOnce(() -> leds.setState(State.PREPARE_SHOT_AMP)),
        elevator
            .extendTheElevator(elevatorHeight.LOW)
            .andThen(new WaitCommand(1.5))
            // .andThen(new WaitUntilCommand(() -> m_elevator.onTarget()))
            .andThen(() -> leds.setState(State.SHOOT_READY_AMP)),
        shooter.setTargetLevel(levelSpeed.LOW));
  }

  public static Command elevatorIntake() {
    return Commands.sequence(
        elevator.extendTheElevator(elevatorHeight.INTAKE),
        Commands.runOnce(() -> leds.setState(State.IDLE)));
  }
}
