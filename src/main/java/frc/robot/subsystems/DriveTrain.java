package frc.robot.subsystems;

import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModule;
import frc.robot.subsystems.Gyro;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  //simulates swerve position from the center of the robot has test values
  Translation2d m_frontLeftPosition = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightPosition = new Translation2d(0.381, -0.381);
  Translation2d m_lowerLeftPosition = new Translation2d(-0.381, 0.381);
  Translation2d m_lowerRightPosition = new Translation2d(-0.381, -0.381);

  private final Gyro m_gyro = new Gyro();
  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftPosition, m_frontRightPosition, m_lowerLeftPosition, m_lowerRightPosition
  );

  private final SwerveDrivePoseEstimator m_odometry =
      new SwerveDrivePoseEstimator(
          m_kinematics, m_gyro.getRotation2d(), this.getModulePosition(), new Pose2d());

      public DriveTrain() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
