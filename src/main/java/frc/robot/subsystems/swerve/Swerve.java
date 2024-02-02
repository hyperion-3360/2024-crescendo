package frc.robot.subsystems.swerve;

import static frc.robot.Constants.WCPSwerveModule.kLocations;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  public SwerveModule[] mSwerveMods;

  public SwerveModulePosition[] positions;

      private final WPI_PigeonIMU m_gyro;

  private final Field2d m_field2d;

  public SwerveDriveOdometry m_odometry;

  public Swerve() {

   m_gyro = new WPI_PigeonIMU(0);

   m_field2d = new Field2d();

    m_gyro.reset();
    m_gyro.enterCalibrationMode(CalibrationMode.Temperature);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
          
        };

        positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    };

         m_odometry = 
  new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, m_gyro.getRotation2d(), positions,
   new Pose2d(0, 0, new Rotation2d()));

  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

     public Pose2d getPose() {
        return m_odometry.getPoseMeters();
   }

     public void setPose(Pose2d pose) {
  m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

     public Rotation2d getHeading(){
        return getPose().getRotation();
   }

     public void setHeading(Rotation2d heading){
         m_odometry.resetPosition(getRotation2d(), getModulePositions(), new
  Pose2d(getPose().getTranslation(), heading));
     }

     public void zeroHeading(){
         m_odometry.resetPosition(getRotation2d(), getModulePositions(), new
  Pose2d(getPose().getTranslation(), new Rotation2d()));
     }
  
     public Rotation2d getRotation2d() {
         return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees());
     }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    //        swerveOdometry.update(getGyroYaw(), getModulePositions());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getMagEncoderPos().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    m_gyro.getRotation2d();

     //updates the odometry positon
     var m_odometryPose = m_odometry.update(m_gyro.getRotation2d(),
     getModulePositions());

     //Renews the field periodically
    m_field2d.setRobotPose(m_odometryPose);

    SmartDashboard.putData(m_field2d);
  }

  public Command resetOdometryBlueSide() {
   return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(2.1, 5, Rotation2d.fromDegrees(180))));
}

public Command resetOdometryRedSide() {
   return this.runOnce(
        () ->
            m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(14.4, 5, Rotation2d.fromDegrees(180))));
  }

public void robotInit() {

  }
}