// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
  public enum Mode { // TODO Ajouter au shuffleboard
    BLUE_AUTO1("Test"),
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

  public static Command followPath(Mode automode) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(automode.toString());
    return AutoBuilder.followPath(path);
  }
}
