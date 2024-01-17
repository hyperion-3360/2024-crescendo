package frc.robot.subsystems;

import static frc.robot.Constants.WCPSwerveModule.kLocations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.Conversions;
import frc.robot.subsystems.WCPSwerveModule.WCPSwerveModuleFactory;

public class DriveTrain extends SubsystemBase {

  private final SwerveModuleFactory m_moduleFactory = new WCPSwerveModuleFactory();
  private final SwerveModule[] m_modules = m_moduleFactory.createModules();

  private SwerveModulePosition[] modulePositions() {
    return m_modules[0].getPositions();
  }

  private final Gyro m_gyro = new Gyro();
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kLocations);

  private final Field2d m_field2d = new Field2d();
  private final SwerveDriveOdometry m_odometry = 
  new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d(), modulePositions()  /*TODO input swerve module position*/,
   new Pose2d(0, 0, new Rotation2d()));

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

     //Renews the field periodically
    m_field2d.setRobotPose(m_odometry.getPoseMeters());
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
                modulePositions(),/*in need of module position*/
                new Pose2d(2.1, 5, Rotation2d.fromDegrees(180))));
}

public Command resetOdometryRedSide() {
   return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                modulePositions(),/*in need of module position*/
                new Pose2d(14.4, 5, Rotation2d.fromDegrees(180))));
  }
public void robotInit(){
  SmartDashboard.putData("Field" ,m_field2d);
  }
}
