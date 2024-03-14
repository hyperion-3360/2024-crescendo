package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;

/** Subsystem for wrapping communication with the vision co-processor */
public class Vision extends SubsystemBase {

  // some local constants
  private final int kAprilTagPosStartIndex = 0;
  private final int kAprilTagRotStartIndex = 3;
  private final int kAprilTagIdsStartIndex = 6;

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
      var rotation = Arrays.copyOfRange(info, kAprilTagRotStartIndex, kAprilTagIdsStartIndex);
      var numTags = info.length - kAprilTagIdsStartIndex;

      var tagRotation = Rotation2d.fromDegrees(rotation[1]);
      var robotRotation = tagRotation;

      m_currentPos = new Pose2d(new Translation2d(position[0], position[1]), robotRotation);
      m_visibleTags = new long[numTags];
      for (int idx = 0; idx < numTags; idx++)
        m_visibleTags[idx] = (long) info[idx + kAprilTagIdsStartIndex];
    } else m_currentPos = null;

    // check if we have a note detected using the AI system
    value = m_detectionValue.getAndSet(null);
    if (value != null) {
      m_detection = new Translation2d(value[0], value[1]);
    } else m_detection = null;
  }

  public Pose2d getPosition() {
    return m_currentPos;
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
    builder.addDoubleProperty("Rotation", () -> getPosition().getRotation().getDegrees(), null);
    builder.addDoubleProperty("Pose X", () -> getPosition().getX(), null);
    builder.addDoubleProperty("Pose Y", () -> getPosition().getY(), null);
    builder.addIntegerArrayProperty("Tag IDs", this::getVisibleTagIds, null);
    builder.addBooleanProperty("Valid Note detection", this::isNoteDetected, null);
    builder.addDoubleProperty("Note detection X", () -> getNoteCoord().getX(), null);
    builder.addDoubleProperty("Note detection Y", () -> getNoteCoord().getY(), null);
  }
}
