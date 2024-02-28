// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConnexions;
import frc.robot.subsystems.Shooter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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

  public static PathPlannerPath[] getSelectedOptionConnexions() {
    // TODO add more pathplanner connexions
    switch (getSelectedOption()) {
      case TwoNotesMidField:
        return PathPlannerConnexions.kCloseUpperNote();
        // case ThreeNotesCenterField:
        //   return PathPlannerConnexions.kCloseUpperNote;
        // case TwoNotesCenterField:
        //   return PathPlannerConnexions.kCloseUpperNote;
        // case FourNotesCenterField:
        //   return PathPlannerConnexions.kCloseUpperNote;
        // case FourNotesFarShot:
        //   return PathPlannerConnexions.kCloseUpperNote;
        // case FourNotesMidField:
        //   return PathPlannerConnexions.kCloseUpperNote;
        // case ThreeNotesMidField:
        //   return PathPlannerConnexions.kCloseUpperNote;
        // case crossRobotZone:
        //   return PathPlannerConnexions.kCloseUpperNote;
      default:
        return null;
    }
  }

  public static class ConditionsMaker {
    // TODO add more conditions to represent what behavior we want the AI to have
    private Shooter m_shooter = new Shooter();

    /**
     * @param timeConstraints at what time treshold do you want this path to not activate ex: after
     *     10 seconds of auto mode there's no way the robot has enough time to do a high shoot so
     *     don't take that path
     * @param wantNote do we want a note before changing path ex: we don't want to intake a note if
     *     there's already one in the intake
     */
    private ConditionsMaker(double timeConstraints, boolean wantNote) {
      timeCondition(timeConstraints);
      noteCondition(wantNote);
    }

    private boolean timeCondition(double timeConstraints) {
      if (timeConstraints > DriverStation.getMatchTime()) {
        return true;
      } else {
        return false;
      }
    }

    private boolean noteCondition(boolean noNote) {
      if (m_shooter.hasNote() != noNote) {
        return true;
      } else {
        return false;
      }
    }

    // TODO implement when AI is done
    // private boolean isNoteGone(boolean noteGone) {
    //   if (Camera.hasNoteInSight == noteGone) {
    //     return true;
    //   } else {
    //     return false;
    //   }
    // }

    public static List<Boolean> setConditions(double timeConstraints, boolean wantNote) {
      return ConditionsMaker.setConditions(timeConstraints, wantNote);
    }
  }

  public static class PathfindingChooser {
    // hash map for the path's index and the connected path to it
    private HashMap<Short, PathPlannerPath> m_pathNodeMap = new HashMap<>();
    // hash map for the connected paths and their conditions
    private static HashMap<Integer, List<Boolean>> m_conditionPerNode = new HashMap<>();
    private static short pathStage = 0;
    private static List<Boolean> conditions = new ArrayList<>();
    private String chosenPathNode;

    // constructor where conditions are fed and accounted for to choose a path
    public PathfindingChooser(String mainPath, PathPlannerPath connexions[]) {
      chosenPathNode = null;
      List<PathPlannerPath> m_autoPath = PathPlannerAuto.getPathGroupFromAutoFile(mainPath);
      conditions.addAll(ConditionsMaker.setConditions(0, false));
      // gives the required conditions to the available function

      for (Integer i = 0; i < connexions.length + m_autoPath.size(); i++) {
        m_pathNodeMap.put(pathStage, connexions[i]);
        m_conditionPerNode.putAll(getConditionPerNodeMap());
        pathNodeChooser(conditions, connexions[i]);
        if (pathNodeChooser(conditions, connexions[i]) != null) {
          break;
        }
      }
    }

    /**
     * handles the logic for the conditions. it will check if ALL the conditions are true. if they
     * aren't and one of them proves to be false it will break from the loop and check the
     * conditions for another path node
     *
     * @param conditions the conditions specific to the connexion we want to check as a boolean list
     * @return the conditions status (if they are all true it'll return true else it returns false)
     */
    private Boolean conditionLogicHandler(List<Boolean> conditions) {
      boolean allConditionReadGreen = false;
      byte trueConditions = 0;
      pathStage = 0;

      for (int i = 0; i < conditions.size(); i++) {
        if (conditions.get(i) == true) {
          ++trueConditions;
          if (trueConditions == conditions.size()) {
            allConditionReadGreen = true;
          }
        } else {
          break;
        }
      }
      if (allConditionReadGreen == true) {
        return true;
      } else {
        return false;
      }
    }

    /**
     * choses a path node if it's condition are met. If not it will delete the path node
     *
     * @param conditions the conditions specific to the connexion we want to check
     * @param connexions the path nodes which will give it's attached condition
     * @return the chosen path node so the pathfinding knows where to go
     */
    public String pathNodeChooser(List<Boolean> conditions, PathPlannerPath connexions) {
      if (conditionLogicHandler(conditions) == true) {
        ++pathStage;
        return chosenPathNode = m_pathNodeMap.values().toString();
      } else {
        m_pathNodeMap.remove(pathStage, connexions);
      }
      return null;
    }

    public static Map<? extends Integer, ? extends List<Boolean>> getConditionPerNodeMap() {
      return m_conditionPerNode;
    }

    public static List<Boolean> setConditionPerNodeMap(
        Integer connexions, List<Boolean> wantedConditions) {
      return m_conditionPerNode.put(connexions, wantedConditions);
    }

    public static short getPathStage() {
      return PathfindingChooser.pathStage;
    }
  }

  public static Command makePathfindingGoToPath() {
    PathfindingChooser m_choosedPath =
        new PathfindingChooser(autoChooser.getSelected().toString(), getSelectedOptionConnexions());
    // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile(m_choosedPath.chosenPathNode);

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
                2.0 // Rotation delay distance in meters. This is how far the robot should travel
                // before
                // attempting to rotate.
                ));
  }
}
