package frc;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardFactory {
    
    public ShuffleboardFactory(){
        Shuffleboard3360.getInstance()
        .addTab("Autonomous mode")
        .addBooleanWidget("Autonomous mode", "Intake mode", false)
        .addPercentWidget("Autonomous mode", "Shooter Hight", 0.0f);
        
                 
    SendableChooser<String> autoChooser;
    
        // Initialiser les objets ici.

        /* Initialiser le tableau de bord Shuffleboard
        Shuffleboard.getTab("Driver")
            .add("LeftEncoder Status", /* Ajoutez l'encodeur ici  )
            .withPosition(0, 0);

             // Initialiser le tableau de bord Shuffleboard
        Shuffleboard.getTab("Driver")
            .add("RightEncoder Status", /* Ajoutez l'encodeur ici  )
            .withPosition(0, 0);*/

        // Initialiser le sélecteur autonome
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Auto Mode 1", "AutoMode1");
        autoChooser.addOption("Auto Mode 2", "AutoMode2");
        Shuffleboard.getTab("Autonomous")
            .add("Auto Mode", autoChooser)
            .withPosition(2, 0);
   
            // Récupérer le mode autonome sélectionné
            String selectedAuto = autoChooser.getSelected();
    
            // Mettre en œuvre la logique autonome ici en fonction de la sélection

            Shuffleboard.getTab("Operator")
                .add("Shooter Height", false)//need checked the defaultValue
                .getEntry("Shooter Height");
               // .setDouble(/* Lire la hauteur du shooter ici */);
        
             // Mettre à jour les valeurs sur le tableau de bord Shuffleboard pendant le mode téléop
            /*Shuffleboard.getTab("Driver")
                .add("LeftENcoder Status",true)// to be checked
                .getEntry("LeftEncoder Status")
                .setDouble(Lire la valeur de l'encodeur ici );
      
             // Mettre à jour les valeurs sur le tableau de bord Shuffleboard pendant le mode téléop
             Shuffleboard.getTab("Driver")
                .getEntry("RightEncoder Status")
                .setDouble( Lire la valeur de l'encodeur ici );*/
        }
}

    
