package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class ModeAuto extends SubsystemBase {
  // TODO Changer pour avoir les bons noms de fichiers
  public enum Mode {
    RED_AUTO1("Red center (mn)"),
    RED_AUTO2("Red right (mn)"),
    RED_AUTO3("Red left (mn)"),
    BLUE_AUTO1("Blue center (mn)"),
    BLUE_AUTO2("Blue right (mn)"),
    BLUE_AUTO3("Blue left (mn)");

    private String m_path;
    private Mode(String path){
      m_path = path;
    }

    public String toString(){
      return m_path;
    }
  }

  public void follow(Mode automode){
    PathPlannerPath path = PathPlannerPath.fromPathFile(automode.toString());
    AutoBuilder.followPath(path);
  }
}
