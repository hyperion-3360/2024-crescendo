package frc.robot.sequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeight;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.State;
import frc.robot.subsystems.Shooter;

public class Shoot {

  private static Shooter shooter;
  private static LEDs leds;
  private static Elevator elevator;

  public Shoot() {
    shoot().addRequirements(shooter, elevator, leds);
  }

  public static Command shoot() {
    return Commands.sequence(
        shooter.shootTo().andThen(() -> leds.setState(State.SHOT_DONE)),
        elevator
            .extendTheElevator(elevatorHeight.INTAKE)
            .andThen(new WaitCommand(2))
            .andThen(() -> leds.setState(State.IDLE)));
  }
}
