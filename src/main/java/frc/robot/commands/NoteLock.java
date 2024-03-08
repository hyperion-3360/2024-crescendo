package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class NoteLock extends Command {
  private Swerve m_swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_lockedOnNote;
  private Translation2d m_target;
  private Vision m_vision;

  public NoteLock(
      Swerve s_swerve, Vision s_vision, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    this.m_swerve = s_swerve;
    this.m_vision = s_vision;
    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;
  }

  // TODO change the x position
  public double getNoteYpos() {
    final var pos = m_vision.getVisibleNotes();
    double xPos = 0;
    if (pos.length == 4) {
      xPos = pos[2];
      xPos = 0.0023 * xPos - 0.6187;
    }
    return xPos;
  }

  // TODO change the y position
  public double getNoteXpos() {
    final var pos = m_vision.getVisibleNotes();
    double yPos = 0;
    if (pos.length == 4) {
      yPos = pos[3];
      yPos = -0.0058 * yPos + 2.782;
    }
    return yPos;
  }

  @Override
  public void execute() {
    // TODO add a way to resolve the issue when two notes share the jetson camera space
    // // if (m_vision.moreThanTwoNotes()) {

    // }

    m_target = new Translation2d(getNoteXpos(), getNoteYpos());

    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal =
        MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband)
            * (getNoteXpos() + getNoteYpos());

    double rotationVal = m_swerve.getRotation2d().getRadians();

    if (m_lockedOnNote) {
      // compute rotation to apply to face the target
      Translation2d pointToFace = m_target;
      Rotation2d rotationNeeded = pointToFace.minus(m_swerve.getPose().getTranslation()).getAngle();
      rotationVal = rotationNeeded.getRadians();
    }

    /* Drive */
    m_swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        false,
        true);
  }
}
