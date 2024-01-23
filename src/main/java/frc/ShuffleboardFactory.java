package frc;

public class ShuffleboardFactory {
    
    public ShuffleboardFactory(){
        Shuffleboard3360.getInstance()
            .addTab("Drivers")
            .addBooleanWidget("Drivers", "Note", false);
    }
}
