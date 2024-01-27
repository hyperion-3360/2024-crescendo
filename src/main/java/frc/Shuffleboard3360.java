package frc;

import java.text.DecimalFormat;
import java.util.Map;
//import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Singleton class to control the Shuffleboard / SmartDashboard display
 */
public class Shuffleboard3360 {
    private static Shuffleboard3360 _instance;

    private Shuffleboard3360(){
        // TODO
    }

    public static Shuffleboard3360 getInstance(){
        if(_instance == null){
            _instance = new Shuffleboard3360();
        }

        return _instance;
    }

    public Shuffleboard3360 addTab(String title){
        Shuffleboard.getTab(title);
        return _instance;
    }

    public Shuffleboard3360 addBooleanWidget(String tabId, String widgetId, boolean defaultValue){
        Shuffleboard.getTab(tabId).add(widgetId, defaultValue)
        .withWidget(BuiltInWidgets.kBooleanBox);

        return _instance;
    }
    public Shuffleboard3360 addPercentWidget(String tabId, String widgetId, Float defaultValue) {
        Shuffleboard.getTab(tabId).add(widgetId, new DecimalFormat("##0.00").format(defaultValue)+" %")
        .withWidget(BuiltInWidgets.kTextView);

        return _instance;

    }


    public void displayData(String title, String widgetId, int[] data){
        // TODO widgetID ????
    }

    public void displayData(String title, String widgetId, double[] data){
        // TODO widgetID ????
    }

    public void displayData(String title, String widgetId, String[] data){
        // TODO widgetID ????
    }
    protected void execute() {

      

       /* SmartDashboard.putNumber("Left Drive Encoder", drivetrain.getLeftEncoder());
        SmartDashboard.putNumber("Right Drive Encoder", drivetrain.getRightEncoder());*/
    }
    public void hasNote() {
            Shuffleboard3360.getInstance()
                .addTab("Drivers")
                .addBooleanWidget("Drivers", "Note", false);
        }
}