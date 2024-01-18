package frc;

/**
 * Singleton class to control the Shuffleboard / SmartDashboard display
 */
public class Shuffleboard {

    private Shuffleboard _instance;

    private Shuffleboard(){
        // TODO
    }

    public static getInstance(){
        if(this._instance == null){
            this._instance = Shuffleboard();
        }

        return this._instance;
    }

    public void addTab(String title){
        // TODO
    }

    public void addWidget(String tabId){
        
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
}
