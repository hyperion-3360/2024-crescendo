// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class Autos {

  public enum Mode {
    FourNotesMidField("4NotesClose"),
    // TwoNotesCenterField("2NotesCenterField"),
    // ThreeNotesCenterField("3NotesCenterField"),
    FourNotesCenterField("4NotesCenterField"),
    TwoNotesCorridor2("2NotesCorridor2"),
    ThreeNotesCorridor3("3NotesCorridor3"),
    ThreeNotesCorridor1("3NotesCorridor1"),
    ShootCrossRobotZone("shootCrossRobotZone"),
    Nothing("nothing"),
    StraightLine("straightLine");

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
    autoChooser.setDefaultOption("score 4 notes close", Mode.FourNotesMidField);

    autoChooser.addOption("score 4 notes close", Mode.FourNotesMidField);
    autoChooser.addOption("score 3 notes corridor 1", Mode.ThreeNotesCorridor1);
    autoChooser.addOption("score 2 notes corridor 2", Mode.TwoNotesCorridor2);
    autoChooser.addOption("score 3 notes corridor 3", Mode.ThreeNotesCorridor3);
    // autoChooser.addOption("score 4 notes center field (meh?)", Mode.FourNotesCenterField);
    autoChooser.addOption("shoot then cross the robot zone", Mode.ShootCrossRobotZone);
    autoChooser.addOption("straight line", Mode.StraightLine);
    autoChooser.addOption("do nothing", Mode.Nothing);

    Shuffleboard.getTab("Autos").add("Auto Mode", autoChooser);

    // Récupérer le mode autonome sélectionné
  }

  public static Mode getSelectedOption() {
    return autoChooser.getSelected();
  }
}
