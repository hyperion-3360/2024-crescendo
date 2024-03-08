package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class NoteLock extends Command {

  private Swerve m_swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_lockedOnNote;
  private Translation2d m_target;
  private Vision m_vision;
  private ArrayList<Translation2d> everynote = new ArrayList<>();

  public NoteLock(
      Swerve s_swerve, Vision s_vision, DoubleSupplier translationSup, DoubleSupplier strafeSup) {

    addRequirements(s_swerve);
    addRequirements(s_vision);

    this.m_swerve = s_swerve;
    this.m_vision = s_vision;
    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;
  }

  @Override
  public void execute() {
    // gives general robot positions and translation/strafe values
    var robotPosition = m_swerve.getPose().getTranslation();
    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = m_swerve.getRotation2d().getRadians();

    if (m_lockedOnNote) {

      // compute variables to feed the drive function of the swerves
      Translation2d pointToFace = m_target;
      Rotation2d rotationNeeded = pointToFace.minus(m_swerve.getPose().getTranslation()).getAngle();
      rotationVal = rotationNeeded.getRadians();

      /* Drive */
      m_swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
          rotationVal * Constants.Swerve.maxAngularVelocity,
          false,
          true);
    } else {
      // checks if the detected note is stale
      if (m_vision.isValidPos()) {
        var note = m_vision.getVisibleNotes();
        var selectedNote = new Translation2d();

        if (note.length == 1) {
          m_lockedOnNote = true;
        }
        // if there's more than one note find the closest one
        else {
          // clears every note from the arraylist upon init
          everynote.clear();
          // instead of a forEach I used a forI to have id info
          for (int i = 0; i < note.length; i++) {
            // adds every translation 2d of every index
            everynote.add(i, new Translation2d(m_vision.getNoteXpos(), m_vision.getNoteYpos()));
            var nearestNote = robotPosition.nearest(everynote);
            selectedNote = nearestNote; // select the nearest note
          }
        }
        var selectedTarget = new Translation2d(m_vision.getNoteXpos(), m_vision.getNoteYpos());

        if (selectedTarget != null) {
          var newTarget = selectedNote;
          m_target = new Translation2d(newTarget.getX(), newTarget.getY());
        }
      }
    }
  }
}
