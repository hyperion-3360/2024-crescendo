// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class Autos {

  public enum Mode {
    BLUE_AUTO1("BlueCenter1"),
    BLUE_AUTO2("BlueRight1"),
    BLUE_AUTO3("BlueLeft1"),
    BLUE_AUTO4("BlueCenter2"),
    BLUE_AUTO5("BlueRight2"),
    BLUE_AUTO6("BlueLeft2");

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
    autoChooser.setDefaultOption(Mode.BLUE_AUTO1.toString(), Mode.BLUE_AUTO1);

    autoChooser.addOption(Mode.BLUE_AUTO2.toString(), Mode.BLUE_AUTO2);
    autoChooser.addOption(Mode.BLUE_AUTO3.toString(), Mode.BLUE_AUTO3);
    autoChooser.addOption(Mode.BLUE_AUTO4.toString(), Mode.BLUE_AUTO4);
    autoChooser.addOption(Mode.BLUE_AUTO5.toString(), Mode.BLUE_AUTO5);
    autoChooser.addOption(Mode.BLUE_AUTO6.toString(), Mode.BLUE_AUTO6);

    Shuffleboard.getTab("Autos").add("Auto Mode", autoChooser);

    // Récupérer le mode autonome sélectionné
  }

  public static Mode getSelectedOption() {
    return autoChooser.getSelected();
  }
}
