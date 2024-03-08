package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Wraps a vision robot pose measurement */
class VisionMeasurement {

  /** Measurement timestamp (s) */
  public long m_bestBefore;

  /** Field relative robot pose (m) */
  public Pose2d m_pose;
}

/** Subsystem for wrapping communication with the vision co-processor */
public class Vision extends SubsystemBase {

  private DoubleArraySubscriber m_camPose =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleArrayTopic("position")
          .subscribe(new double[] {});

  private DoubleArraySubscriber m_camRotation =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getDoubleArrayTopic("rotation")
          .subscribe(new double[] {});

  private IntegerArraySubscriber m_tagsDetected =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getIntegerArrayTopic("tagIds")
          .subscribe(new long[] {});

  // private DoubleArraySubscriber m_detection =
  //    NetworkTableInstance.getDefault()
  //        .getTable("SmartDashboard")
  //        .getDoubleArrayTopic("detection")
  //        .subscribe(new double[] {});

  private VisionMeasurement m_currentPos;
  private long[] m_visibleTags;

  /** Creates a new Vision. */
  public Vision() {
    m_currentPos.m_bestBefore = -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    final var position = m_camPose.get();
    final var rotation = m_camRotation.get();

    if (position.length == 3 && rotation.length == 3) {
      var tagRotation = Rotation2d.fromDegrees(rotation[1]);
      var robotRotation = tagRotation;
      var measurement = new VisionMeasurement();

      measurement.m_bestBefore =
          RobotController.getFPGATime() + Constants.kVisionPositionCoalescingTime;
      measurement.m_pose = new Pose2d(new Translation2d(position[0], position[1]), robotRotation);
      m_visibleTags = m_tagsDetected.get();
    } else { // the detection is not going to be meaningfull forever... the coalescing time added at
      // detection is checked here and the entire measurement is invalidated if too old
      if ((m_currentPos.m_bestBefore != -1)
          && (m_currentPos.m_bestBefore > RobotController.getFPGATime()))
        m_currentPos.m_bestBefore = -1;
    }
  }

  /**
   * Get the latest available measurement from the vision system
   *
   * @return latest measurement or null
   */
  public Pose2d getPosition() {
    return m_currentPos.m_pose;
  }

  public long[] getVisibleTagIds() {
    return m_visibleTags;
  }

  public boolean isValidPos() {
    return m_currentPos.m_bestBefore != -1;
  }
}
