// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;

/** Add your docs here. */
public class automod extends SubsystemBase {
  private Supplier<Pose2d> m_poseSupplier;
  private Consumer<Pose2d> m_poseResetConsumer;
  private Supplier<ChassisSpeeds> getRobotRelativeSpeeds;
  private Consumer<ChassisSpeeds> driveRobotRelative;

  // TODO Changer pour avoir les bons noms de fichiers
  public enum Mode {
    RED_AUTO1("Left 1.auto"),
    RED_AUTO2("Center 1.auto"),
    RED_AUTO3(""),
    BLUE_AUTO1(""),
    BLUE_AUTO2(""),
    BLUE_AUTO3("");

    private String m_path;
    private Mode(String path){
      m_path = path;
    }

    public String toString(){
      return m_path;
    }
  }

  public automod(Swerve drivetrain, PIDConstants swervesPid, PIDConstants rotationPid) {
    // Init the pose and speed suppliers and consumers for pathplanner...
    m_poseSupplier = () -> drivetrain.getPose();
    m_poseResetConsumer = (Pose2d new_zero) -> drivetrain.setPose(new_zero);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this.m_poseSupplier, // Robot pose supplier
        this.m_poseResetConsumer, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this.getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this.driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            swervesPid, // Translation PID constants
            rotationPid, // Rotation PID constants
            Constants.Swerve.maxSpeed, // Max module speed, in m/s
            Constants.robotBaseRadiusApprox, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public void follow(Mode automode){
    PathPlannerPath path = PathPlannerPath.fromPathFile(automode.toString());
    AutoBuilder.followPath(path);
  }
}
