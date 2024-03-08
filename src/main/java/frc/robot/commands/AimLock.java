package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public class AimLock extends Command {
  private Swerve m_swerve;
  private Elevator m_elevator;
  private DoubleSupplier m_translationSup;
  private DoubleSupplier m_strafeSup;
  private boolean m_isLocked;
  private Translation2d m_target;
  private double m_targetHeight;
  private LEDs m_led;
  private Vision m_vision;
  private AprilTagFieldLayout m_aprilTagFieldLayout;

  /**
   * Command to keep the aim of a target while keeping the robot in motion
   *
   * @param s_swerve swerve submodule instance
   * @param s_elevator elevator submodule instance
   * @param s_led led submodule instance
   * @param s_vision vision submodule instance
   * @param translationSup translation forward or backward
   * @param strafeSup moving laterally
   */
  public AimLock(
      Swerve s_swerve,
      Elevator s_elevator,
      LEDs s_led,
      Vision s_vision,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup) {

    this.m_swerve = s_swerve;
    this.m_elevator = s_elevator;
    this.m_led = s_led;
    this.m_vision = s_vision;

    try {
      m_aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      m_aprilTagFieldLayout = null;
    }

    addRequirements(s_swerve);
    addRequirements(s_elevator);
    addRequirements(s_led);
    addRequirements(s_vision);

    this.m_translationSup = translationSup;
    this.m_strafeSup = strafeSup;
    this.m_isLocked = false;
  }

  @Override
  public void execute() {
    double translationVal =
        MathUtil.applyDeadband(m_translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), Constants.stickDeadband);

    double rotationVal = m_swerve.getRotation2d().getRadians();

    var cur_pos = m_swerve.getPose().getTranslation();

    if (m_isLocked) {
      // compute rotation to apply to face the target
      Translation2d pointToFace = m_target;
      Rotation2d rotationNeeded = pointToFace.minus(m_swerve.getPose().getTranslation()).getAngle();
      rotationVal = rotationNeeded.getRadians();

      // compute elevator angle to shoot in target
      double distance2Target = cur_pos.getDistance(pointToFace);
      if (distance2Target > Constants.ShooterConstants.kMaxShootingDistance)
        m_led.setState(LEDs.State.PREPARE_SHOT_SPEAKER);
      else {
        double elevatorAngle = Math.atan2(m_targetHeight, distance2Target);
        m_elevator.extendTheElevator(elevatorAngle);
        m_led.setState(LEDs.State.SHOOT_READY_SPEAKER);
      }
    }
    // look at possible target from vision
    else {
      if (m_vision.isValidPos()) {

        var tags = m_vision.getVisibleTagIds();
        int selecteg_tag = -1;

        if (tags.length == 1) { // well that's the only tag we have
          selecteg_tag = (int) tags[0];
        } else { // find the closest one
          double closerDistance = Double.MAX_VALUE;
          for (var tagId : m_vision.getVisibleTagIds()) {
            var tagPose = m_aprilTagFieldLayout.getTagPose((int) tagId).get();
            // no need for Z
            var xyTagPose = new Translation2d(tagPose.getX(), tagPose.getY());
            var distance2Tag = xyTagPose.getDistance(cur_pos);
            if (distance2Tag < closerDistance) {
              closerDistance = distance2Tag;
              selecteg_tag = (int) tagId;
            }
          }
        }
        // selected_tag is necessary different than -1 but better safe than sorry
        var selectedTarget = m_aprilTagFieldLayout.getTagPose(selecteg_tag);
        if (selectedTarget.isPresent()) {
          // get rid of Z
          var newTarget = m_aprilTagFieldLayout.getTagPose(selecteg_tag).get();
          m_target = new Translation2d(newTarget.getX(), newTarget.getY());
        }
      }
    }

    /* Drive */
    m_swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        false,
        true);
  }
}
