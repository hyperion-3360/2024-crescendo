package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.Swerve;
import java.io.IOException;
import java.util.Arrays;

public class AmpLock extends Command {
  private Swerve m_swerve;
  private Elevator m_elevator;
  private boolean m_isLocked;
  private Translation2d m_target;
  // private double m_targetRotation;
  private LEDs m_led;
  private Vision m_vision;
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  private Translation2d m_aprilTags[];
  // private double m_aprilTagsRotation[];
  private int m_alliance_index;

  /**
   * Command to drive the robot towards the AMP automatically
   *
   * @param s_swerve swerve submodule instance
   * @param s_elevator elevator submodule instance
   * @param s_led led submodule instance
   * @param s_vision vision submodule instance
   */
  public AmpLock(Swerve s_swerve, Elevator s_elevator, LEDs s_led, Vision s_vision) {

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

    if (m_aprilTagFieldLayout != null) {
      m_aprilTags = new Translation2d[Constants.VisionConstants.kAmpIndex.length];
      // m_aprilTagsRotation = new double[Constants.VisionConstants.kAmpIndex.length];

      // initialize translation2d objects for each AprilTag on the field
      int i = 0;
      for (var tagId : Constants.VisionConstants.kAmpIndex) {
        var tagPose = m_aprilTagFieldLayout.getTagPose(tagId).get();
        // var tagRot = tagPose.getRotation();

        // no need for Z
        m_aprilTags[i] = new Translation2d(tagPose.getX(), tagPose.getY());
        // m_aprilTagsRotation[i] = tagRot.getZ();

        i++;
      }

      // fail safe
      m_alliance_index = 0;
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) m_alliance_index = alliance.get() == Alliance.Red ? 0 : 1;
    }

    addRequirements(s_swerve);
    addRequirements(s_elevator);
    addRequirements(s_led);
    addRequirements(s_vision);

    this.m_isLocked = false;
  }

  @Override
  public void initialize() {
    m_isLocked = false;
  }

  @Override
  public void execute() {

    if (m_aprilTagFieldLayout != null) {

      // double rotationVal = m_swerve.getRotation2d().getRadians();

      // var cur_pos = m_swerve.getPose().getTranslation();

      if (m_isLocked && m_vision.getPosition() != null) {
        final var cameraPose2d = m_vision.getPosition();
        var camTranslation = new Translation2d(cameraPose2d.getX(), cameraPose2d.getY());

        // compute rotation to apply to face the target
        Translation2d pointToFace = m_target;
        // double tagOrientation = m_targetRotation;

        // distance from camera to tag -- get distance from robot
        final var dx = camTranslation.getX() - pointToFace.getX();
        final var dy = camTranslation.getY() - pointToFace.getY();
        final var hyp = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        final var distanceFromTag = Math.sin(Constants.VisionConstants.kShooterCameraPitch) * hyp;

        // Calulate the rotation needed to face the tag
        // Correct for camera position. Anticlockwise rotatiozn
        double camRot = cameraPose2d.getRotation().getRadians();
        double rotationVal = camRot + Constants.VisionConstants.kShooterCameraPitch;

        SmartDashboard.putNumber("Distance from target", distanceFromTag);
        SmartDashboard.putNumber("Rotation needed (rad)", rotationVal);

        // compute elevator angle to shoot in target, assuming straightline not parabol
        double distance2Target = distanceFromTag; // cur_pos.getDistance(pointToFace);
        if (distance2Target <= Constants.VisionConstants.kAmpRiseElevatorDistance) {
          m_elevator.extendTheElevator(Elevator.elevatorHeight.LOW);
        }
        if (distance2Target <= Constants.VisionConstants.kAmpShootingDistance) {
          m_led.setState(LEDs.State.SHOOT_READY_SPEAKER);
        } else {
          // Reset the odometry X,Y positioning before moving
          // to make sure the robot knows where it is on then field !!
          m_vision.resetOdometryTranslation2d(m_swerve);

          /* Drive */
          m_swerve.drive(
              pointToFace.times(Constants.Swerve.maxSpeed),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              false,
              false);
        }
      } else { // don't have a lock yet so look at possible target from vision
        int selecteg_tag = -1;
        if (m_vision.isValidPos()) {
          for (var t : m_vision.getVisibleTagIds())
            if (t == Constants.VisionConstants.kAmpIndex[m_alliance_index]) {
              selecteg_tag = Constants.VisionConstants.kAmpIndex[m_alliance_index];
              break;
            }

          if (selecteg_tag != -1) {
            // TODO: optimize this.. Meh recreating object each time...¯\_(ツ)_/¯
            final var tagIdx =
                Arrays.asList(Constants.VisionConstants.kAmpIndex).indexOf(selecteg_tag);
            m_target = m_aprilTags[tagIdx];
            // m_targetRotation = m_aprilTagsRotation[tagIdx];
            m_isLocked = true;
            m_led.setState(LEDs.State.PREPARE_SHOT_SPEAKER);
          } else m_led.setState(LEDs.State.IDLE);
        }
      }
    }
  }
}
