package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.net.PortForwarder;


public class Limelight extends SubsystemBase {

    GenericEntry apriltagEntry = Shuffleboard.getTab("gapril tag")
                                                .add("april tag data", new double[]{0.0,0.0,0.0})
                                                .getEntry();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");


    public void robotInit()
    {
        // Make sure you only configure port forwarding once in your robot code.
        // Do not place these function calls in any periodic functions
        for (int port = 5801; port <= 5805; port++) {
            PortForwarder.add(port, "limelight.local", port);
        
            // Exemple pour obtenir l'offset horizontal
           // double horizontalOffset = Limelight.getHorizontalOffset();
        
        }

        LimelightLEDControl.main();
    }

    public void periodic(){
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        apriltagEntry.setDoubleArray(new double[]{x, y, area});

    }


public class LimelightLEDControl {

    public static void main() {
        // Initialise la connexion avec le réseau de la Limelight
       NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        ntInst.startDSClient(5802); 

        // Attend que la connexion soit établie (c'est souvent une bonne idée de le faire)
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } 

        // Active les LED
        setLimelightLEDState(true);

        // Faites d'autres choses ici...

        // Désactive les LED à la fin du programme
       // setLimelightLEDState(false);

        // Arrête la connexion avec le réseau de la Limelight
        //ntInst.stopClient();
    }

    // Méthode pour activer ou désactiver les LED de la Limelight
    private static void setLimelightLEDState(boolean state) {
        // Utilisez la méthode setLEDMode() avec le paramètre Limelight.LEDMode.ON ou Limelight.LEDMode.OFF
        // en fonction de l'état que vous souhaitez
     //Limelight.setLEDMode(state ? Limelight.LEDMode.ON : Limelight.LEDMode.OFF);
      // Obtient la table Limelight
      NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

      // Utilise la clé "ledMode" pour contrôler l'état des LED (0 pour éteint, 1 pour allumé)
      limelightTable.getEntry("ledMode").setNumber(state ? 3 : 1);
    }
}
}