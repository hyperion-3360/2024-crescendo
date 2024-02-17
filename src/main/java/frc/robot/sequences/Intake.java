package frc.robot.sequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.State;
import frc.robot.subsystems.Shooter;

public class Intake {

  private static Shooter shooter;
  private static LEDs leds;

  public Intake() {
    intakeSequence().addRequirements(shooter, leds);
  }

  public static Command intakeSequence() {
    return Commands.sequence(
        Commands.runOnce(() -> leds.setState(State.INTAKE_ROLLING)),
        shooter
            .intake()
            .andThen(() -> leds.setState(State.NOTE_INSIDE))
            .andThen(new WaitCommand(2).andThen(() -> leds.setState(State.IDLE))));
  }
}
