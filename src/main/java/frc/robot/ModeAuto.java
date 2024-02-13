package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ModeAuto extends SubsystemBase {
  // TODO Changer pour avoir les bons noms de fichiers
  public enum Mode {
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

  public void follow(Mode automode) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(automode.toString());
    AutoBuilder.followPath(path);
  }
}
