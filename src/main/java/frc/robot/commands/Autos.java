// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;

public final class Autos {

  public enum Mode {
    TwoNotesMidField("2NotesMidField"),
    ThreeNotesMidField("3NotesMidField"),
    FourNotesMidField("4NotesMidField"),
    TwoNotesCenterField("2NotesCenterField"),
    ThreeNotesCenterField("3NotesCenterField"),
    FourNotesCenterField("4NotesCenterField"),
    crossRobotZone("crossRobotZone"),
    FourNotesFarShot("4NotesFarShot");

    private String m_path;

    private Mode(String path) {
      m_path = path;
    }

    public String toString() {
      return m_path;
    }
  }

  // public static Command followPath(Mode automode) {
  //   PathPlannerPath path = PathPlannerPath.fromPathFile("Test");

  //   return AutoBuilder.followPath(path);
  // }

  private static SendableChooser<Mode> autoChooser = new SendableChooser<>();

  public static void setShuffleboardOptions() {
    autoChooser.setDefaultOption("score 2 notes mid field", Mode.TwoNotesMidField);

    autoChooser.addOption("score 2 notes mid field", Mode.TwoNotesMidField);
    autoChooser.addOption("score 3 notes mid field", Mode.ThreeNotesMidField);
    autoChooser.addOption("score 4 notes mid field", Mode.FourNotesMidField);
    autoChooser.addOption("score 2 notes center field", Mode.TwoNotesCenterField);
    autoChooser.addOption("score 3 notes center field", Mode.ThreeNotesCenterField);
    autoChooser.addOption("score 4 notes center field", Mode.FourNotesCenterField);
    autoChooser.addOption("cross the robot zone", Mode.crossRobotZone);
    autoChooser.addOption("4 notes far shot", Mode.FourNotesFarShot);

    Shuffleboard.getTab("Autos").add("Auto Mode", autoChooser);

    // Récupérer le mode autonome sélectionné
  }

  public static Mode getSelectedOption() {
    return autoChooser.getSelected();
  }

  private void desirabilityCalculator() {
    PathPlannerPath pathNotePostions[] = {};
    ArrayList<PathPlannerPath> m_pointsOfInterest = new ArrayList<PathPlannerPath>();
    m_pointsOfInterest.add(PathPlannerPath.fromPathFile("speaker path")); // speaker path postion
    m_pointsOfInterest.add(PathPlannerPath.fromPathFile("amp path")); // amp path postion
    // adds every note postions on the array list
    for (int i = 0; i < pathNotePostions.length; i++) {
      m_pointsOfInterest.add(pathNotePostions[i]);
    }
  }

  public static Command makePathfindingGoToPath() {
    // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile("place holder path");

    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only be used for the path.
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return Commands.runOnce(
        () ->
            AutoBuilder.pathfindThenFollowPath(
                path,
                constraints,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel
                // before
                // attempting to rotate.
                ));
  }
}
