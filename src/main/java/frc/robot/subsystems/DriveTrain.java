package frc.robot.subsystems;

import static frc.robot.Constants.WCPSwerveModule.kLocations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final Gyro m_gyro = new Gyro();
  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kLocations);

  private final SwerveDriveOdometry m_odometry = 
  new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), SwerveModulePosition[] /*TODO input swerve module position*/,
   new Pose2d(0, 0/*random values may change*/, new Rotation2d()));

      public DriveTrain() {
        //reset the gyro because odometry start is 0
        m_gyro.gyroCalibrate();
        m_gyro.gyroReset();
      }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic(){
    //updates the odometry positon
    var m_odometryPose = m_odometry.update(m_gyro.getRotation2d(),
     new SwerveModulePosition[] {
      //TODO input swerve module
     });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

public Command resetOdometryBlueSide() {
   return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                getModulePositions(),/*in need of module position*/
                new Pose2d(2.1, 5, Rotation2d.fromDegrees(180))));
}

public Command resetOdometryRedSide() {
   return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                getModulePositions(),/*in need of module position*/
                new Pose2d(14.4, 5, Rotation2d.fromDegrees(180))));
  }
}
