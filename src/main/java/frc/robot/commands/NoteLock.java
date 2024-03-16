package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.util.List;
import java.util.function.DoubleSupplier;

public class NoteLock extends Command {

  private Swerve m_swerve;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_lockedOnNote;
  private Translation2d m_target;
  private Vision m_vision;
  // debug variables
  private boolean debugging;
  private Field2d m_debugField2d;

  /**
   * constructor of the notelock class needs the subsystems and the axis required
   *
   * @param s_swerve the swerve subsystem
   * @param s_vision the vision subsystem
   * @param translationSup the translation axis required
   * @param strafeSup the strafe axis required
   */
  public NoteLock(
      Swerve s_swerve, Vision s_vision, DoubleSupplier translationSup, DoubleSupplier strafeSup) {

    addRequirements(s_swerve);
    addRequirements(s_vision);

    this.m_swerve = s_swerve;
    this.m_vision = s_vision;
    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;

    // debug calling
    debugging = false;
  }

  @Override
  public void execute() {
    // gives general robot positions and translation/strafe values
    double neededAngle;
    var robotPosition = m_swerve.getPose().getTranslation();
    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = m_swerve.getRotation2d().getRadians();

    // #region "debug mini test unit"

    if (debugging == true) {
      debug();

      neededAngle =
          getDebugTanOfY(
              m_target.getX(), m_target.getY(), robotPosition.getX(), robotPosition.getY());

      var m_trajectory =
          TrajectoryGenerator.generateTrajectory(
              new Pose2d(robotPosition, Rotation2d.fromRadians(rotationVal)),
              List.of(
                  robotPosition.plus(new Translation2d(1, 1)),
                  m_target.minus(new Translation2d(1, 1))),
              new Pose2d(m_target, Rotation2d.fromRadians(rotationVal)),
              new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
      m_debugField2d.getObject("center point").setPose(new Pose2d(m_target, new Rotation2d(0)));
      m_debugField2d.getObject("trajectory").setTrajectory(m_trajectory);

      SmartDashboard.putData(m_debugField2d);
    } else
    // #endregion
    if (m_lockedOnNote) {

      neededAngle = getTanOfY(m_target.getX(), m_target.getY());
      SmartDashboard.putNumber("needed angle", neededAngle);

      // compute variables to feed the drive function of the swerves
      Translation2d pointToFace = m_target;
      Rotation2d rotationNeeded = pointToFace.minus(m_swerve.getPose().getTranslation()).getAngle();
      rotationVal = rotationNeeded.getRadians();

      /* Drive */
      m_swerve.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
          (rotationVal + neededAngle) * Constants.Swerve.maxAngularVelocity,
          // (rotationVal + neededAngle) * Constants.Swerve.maxAngularVelocity,
          false,
          true);
    } else {
      // checks if the detected note is stale
      if (m_vision.isValidPos() == true && debugging == false) {
        m_target = m_vision.getNoteBoundingBox();
        // checks if the vision has a note targeted before setting m_lockedOnNote to true
        m_lockedOnNote =
            (m_lockedOnNote == false)
                ? (!m_target.equals(new Translation2d(-1, -1))) ? true : false
                : true;
      }
      if (DriverStation.isAutonomous()) {
        if (m_target.getX() == 0.5) {
          m_swerve.drive(
              new Translation2d(translationVal, strafeVal)
                  .times(Constants.Swerve.maxSpeed)
                  .plus(new Translation2d(robotPosition.plus(new Translation2d(5, 0)).getX(), 0)),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              false,
              true);
        }
      }
    }
  }

  private double getTanOfY(double x, double y) {
    /* AS is the adjacent side and OS is the opposite side */
    var AS = 1 - y;
    var OS = 0.5 - x;
    return Math.atan2(OS, AS);
  }

  private double getDebugTanOfY(double x, double y, double robotX, double robotY) {
    /* AS is the adjacent side and OS is the opposite side */
    var AS = robotY - y;
    var OS = robotX - x;
    return Math.atan2(OS, AS);
  }

  private void debug() {
    m_lockedOnNote = false;
    m_target = new Translation2d(11, 12);
  }
}
