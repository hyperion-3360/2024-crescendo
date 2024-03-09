package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class NoteLockWithBoundingBox extends Command {

  private Swerve m_swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_lockedOnNote;
  private Translation2d m_target;
  private Vision m_vision;

  /**
   * constructor of the notelock class needs the subsystems and the axis required
   *
   * @param s_swerve the swerve subsystem
   * @param s_vision the vision subsystem
   * @param translationSup the translation axis required
   * @param strafeSup the strafe axis required
   */
  public NoteLockWithBoundingBox(
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
        m_target = m_vision.getNoteBoundingBox();
        var neededAngle =
            getSinOfY(m_vision.getNoteBoundingBox().getX(), m_vision.getNoteBoundingBox().getY());
        m_lockedOnNote = true;
      }

      /* concept determine that the left of the bounding box is 0 and right 1 in x
       * and that the upper pos is 0 and down is 1 in y
       * use the 0.5 coordinate of both aka the middle of the bounding box
       * to determine that the note is indeed facing the middle of the robot*/
    }
  }

  // private double xDistance(double distanceBetweenTheTwoPoints, double yCoordinate) {
  //   return Math.sqrt(distanceBetweenTheTwoPoints + yCoordinate);
  // }

  private double getSinOfY(double x, double y) {
    /*AS is the AS is the adjacent side and OS is the opposite side */
    var AS = 1 - y;
    var OS = 0.5 - x;
    return Math.atan2(OS, AS);
  }
}
