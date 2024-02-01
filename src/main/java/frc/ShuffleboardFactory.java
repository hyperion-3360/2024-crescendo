package frc;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.text.DecimalFormat;

/** Singleton class to control the Shuffleboard / SmartDashboard display */
public class ShuffleboardFactory {

  public ShuffleboardFactory addTab(String title) {
    Shuffleboard.getTab(title);
    return this;
  }

  public ShuffleboardFactory addBooleanWidget(String tabId, String widgetId, boolean defaultValue) {
    Shuffleboard.getTab(tabId).add(widgetId, defaultValue).withWidget(BuiltInWidgets.kBooleanBox);

    return this;
  }

  public ShuffleboardFactory addPercentWidget(String tabId, String widgetId, Float defaultValue) {
    Shuffleboard.getTab(tabId)
        .add(widgetId, new DecimalFormat("##0.00").format(defaultValue) + " %")
        .withWidget(BuiltInWidgets.kTextView);

    return this;
  }

  public ShuffleboardFactory addSelector(String tabId, String widgetId, String[] options) {
    if (options.length > 0) {
      Integer c = 0; // Ensure we have unique ids for each option

      SendableChooser<String> chooser = new SendableChooser<>();

      chooser.setDefaultOption(options[0], options[0].replaceAll("\\s+", "") + (c++).toString());

      for (int i = 1; i < options.length; ++i) {
        chooser.addOption(options[i], options[i].replaceAll("\\s+", "") + (c++).toString());
      }

      Shuffleboard.getTab(tabId).add(widgetId, chooser);
    }

    return this;
  }
}
