package frc;

public class Shuffleboard3360 {

  public static class Tabs {
    public static final String drivers = "Operator";
    public static final String auto = "Auto mode";
  }

  public static class Widgets {
    public static final String intakeMode = "Intake mode";
    public static final String shooterHeight = "Shooter height";
    public static final String hasNote = "Has note";
  }

  private static Shuffleboard3360 m_instance;

  public static Shuffleboard3360 getInstance() {
    if (m_instance == null) {
      m_instance = new Shuffleboard3360();
    }

    return m_instance;
  }

  private Shuffleboard3360() {
    ShuffleboardFactory shuffleboard = new ShuffleboardFactory();

    shuffleboard
        .addTab(Tabs.drivers)
        .addBooleanWidget(Tabs.drivers, Widgets.intakeMode, false)
        .addPercentWidget(Tabs.drivers, Widgets.shooterHeight, 0.0f)
        .addBooleanWidget(Tabs.drivers, Widgets.hasNote, false);

    shuffleboard
        .addTab(Tabs.auto)
        .addSelector(
            Tabs.auto,
            "Auto mode",
            new String[] {
              "Mode auto 1", "Mode auto 2",
            });

    // SendableChooser<String> autoChooser;

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
    // autoChooser = new SendableChooser<>();
    // autoChooser.setDefaultOption("Auto Mode 1", "AutoMode1");
    // autoChooser.addOption("Auto Mode 2", "AutoMode2");
    // Shuffleboard.getTab(Tabs.auto)
    //     .add("Auto Mode", autoChooser)
    //     .withPosition(2, 0);

    //     // Récupérer le mode autonome sélectionné
    //     String selectedAuto = autoChooser.getSelected();

    // Mettre en œuvre la logique autonome ici en fonction de la sélection

    /*Shuffleboard.getTab("Operator")
     .add("Shooter Height", false)//need checked the defaultValue
     .getEntry("Shooter Height");
    // .setDouble(/* Lire la hauteur du shooter ici );*/

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

  public void displayData(String title, String widgetId, int[] data) {
    // TODO widgetID ????
  }

  public void displayData(String title, String widgetId, double[] data) {
    // TODO widgetID ????
  }

  public void displayData(String title, String widgetId, String[] data) {
    // TODO widgetID ????
  }

  protected void execute() {

    /* SmartDashboard.putNumber("Left Drive Encoder", drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("Right Drive Encoder", drivetrain.getRightEncoder());*/
  }

  public void hasNote() {}
}
