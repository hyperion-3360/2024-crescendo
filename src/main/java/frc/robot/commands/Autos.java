// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class Autos {

  public enum Mode {
    TwoNotesMidField("2NotesMidField"),
    ThreeNotesMidField("3NotesMidField"),
    FourNotesMidField("4NotesMidField"),
    TwoNotesCenterField("2NotesCenterField"),
    ThreeNotesCenterField("3NotesCenterField"),
    FourNotesCenterField("4NotesCenterField"),
    CrossRobotZone("crossRobotZone"),
    FourNotesMidFieldStartCenter("4NotesMidFieldStartCenter");

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
    autoChooser.addOption("cross the robot zone", Mode.CrossRobotZone);
    autoChooser.addOption("score 4 Notes Mid Field Start Center", Mode.FourNotesMidFieldStartCenter);

    Shuffleboard.getTab("Autos").add("Auto Mode", autoChooser);

    // Récupérer le mode autonome sélectionné
  }

  public static Mode getSelectedOption() {
    return autoChooser.getSelected();
  }
}
