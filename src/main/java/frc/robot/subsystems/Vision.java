package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;

/** Subsystem for wrapping communication with the vision co-processor */
public class Vision extends SubsystemBase {

  // some local constants
  private final int kAprilTagPosStartIndex = 0;
  private final int kAprilTagRotStartIndex = 3;
  private final int kAprilTagAnglesStartIndex = 6;
  private final int kAprilTagIdsStartIndex = 9;

  private final double kCoalescingTime = 0.25; // seconds
  private double m_taglastUpdate = 0;

  final AtomicReference<double[]> m_tagValue = new AtomicReference<double[]>();
  final AtomicReference<double[]> m_detectionValue = new AtomicReference<double[]>();

  private DoubleArraySubscriber m_aprilTagInfo =
      NetworkTableInstance.getDefault()
          .getTable("Vision")
          .getDoubleArrayTopic("position")
          .subscribe(new double[] {});

  private DoubleArraySubscriber m_detectionInfo =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleArrayTopic("detection")
          .subscribe(new double[] {});

  private Pose2d m_currentPos;
  private long[] m_visibleTags;
  private Translation2d m_detection;

  /** Creates a new Vision. */
  public Vision() {
    m_currentPos = null;

    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "Vision"
    NetworkTable visionTable = inst.getTable("Vision");

    // subscribe to the topic in "datatable" called "AprilTags"
    m_aprilTagInfo = visionTable.getDoubleArrayTopic("AprilTags").subscribe(new double[] {});

    // add a listener to get notified only when AprilTags changes
    inst.addListener(
        m_aprilTagInfo,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          m_tagValue.set(event.valueData.value.getDoubleArray());
        });

    // subscribe to the topic in "datatable" called "AprilTags"
    m_detectionInfo = visionTable.getDoubleArrayTopic("Note").subscribe(new double[] {});

    // add a listener to get notified only when AprilTags changes
    inst.addListener(
        m_detectionInfo,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          m_detectionValue.set(event.valueData.value.getDoubleArray());
        });
  }

  @Override
  public void periodic() {
    // get the latest value by reading the AtomicReference; set it to null
    // when we read to ensure we only get value changes
    double[] value = m_tagValue.getAndSet(null);

    // if we have a value to process
    if (value != null) {
      var info = m_aprilTagInfo.get();

      // for convenience just copy slice of the bigger array into smaller ones
      var position = Arrays.copyOfRange(info, kAprilTagPosStartIndex, kAprilTagRotStartIndex);
      var rotation = Arrays.copyOfRange(info, kAprilTagRotStartIndex, kAprilTagAnglesStartIndex);
      var numTags = info.length - kAprilTagIdsStartIndex;
      var angles = Arrays.copyOfRange(info, kAprilTagAnglesStartIndex, kAprilTagIdsStartIndex);

      // Relative orientation between the camera and the april tag
      // Rotation around the vertical axis is at index 1
      // TODO: Jetson script returns an average value of these orientatione.
      //  Not valid if more than 1 camera
      var tagRotation = Rotation2d.fromDegrees(angles[1]);

      // TODO: rotation calculated by jetson script doesn't seem valid..
      // Can't rely on it for now. using tagRotation instead. tbd
      var robotRotation = Rotation2d.fromDegrees(rotation[2]);

      m_currentPos = new Pose2d(new Translation2d(position[0], position[1]), tagRotation);
      m_visibleTags = new long[numTags];
      for (int idx = 0; idx < numTags; idx++)
        m_visibleTags[idx] = (long) info[idx + kAprilTagIdsStartIndex];

      m_taglastUpdate = Timer.getFPGATimestamp();
    } else if (Timer.getFPGATimestamp() - m_taglastUpdate > kCoalescingTime) {
      m_currentPos = null;
    }

    // check if we have a note detected using the AI system
    value = m_detectionValue.getAndSet(null);
    if (value != null) {
      m_detection = new Translation2d(value[0], value[1]);
    } else {
      m_detection = null;
    }
  }

  /**
   * POSE2D OF THE CAMERA !!!!!!!!!!! Robot positionning needs to take into account the camera's
   * pitch, yaw, roll and relative position in robot
   *
   * @return Camera position and rotation in field
   */
  public Pose2d getPosition() {
    return m_currentPos;
  }

  /**
   * Can be used to reset the robot's positioning in the field, (but not the heading ), based on the
   * detected april tags TODO: Automatically reset the odometry instead of calling it manually, once
   * the april tags values are more stable and robust...
   *
   * @param s_swerve Swerves subsystem
   */
  public void resetOdometryTranslation2d(Swerve s_swerve) {
    if (isValidPos()) {
      final var currentRobotPose = s_swerve.getPose().getTranslation();

      final var tagPose = getPosition();
      final var resetPose =
          new Pose2d(
              new Translation2d(tagPose.getX(), tagPose.getY()), currentRobotPose.getAngle());

      s_swerve.setPose(resetPose);
    }
  }

  public Translation2d getNoteCoord() {
    return m_detection;
  }

  public long[] getVisibleTagIds() {
    return m_visibleTags;
  }

  public boolean isNoteDetected() {
    return m_detection != null;
  }

  public boolean isValidPos() {
    return m_currentPos != null;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");
    builder.addBooleanProperty("Valid AprilTag", this::isValidPos, null);
    if (m_currentPos != null) {
      builder.addDoubleProperty("Rotation", () -> getPosition().getRotation().getDegrees(), null);
      builder.addDoubleProperty("Pose X", () -> getPosition().getX(), null);
      builder.addDoubleProperty("Pose Y", () -> getPosition().getY(), null);
      builder.addIntegerArrayProperty("Tag IDs", this::getVisibleTagIds, null);
    }
    // builder.addBooleanProperty("Valid Note detection", this::isNoteDetected, null);
    // if(getNoteCoord() != null){
    //   builder.addDoubleProperty("Note detection X", () -> getNoteCoord().getX(), null);
    //   builder.addDoubleProperty("Note detection Y", () -> getNoteCoord().getY(), null);
    // }
  }
}
