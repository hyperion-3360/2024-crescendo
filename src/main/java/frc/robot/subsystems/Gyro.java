package frc.robot.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    // Make a new pigeon
    WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);

    // Display the yaw pitch and roll on the ShuffleBoard
    GenericEntry m_gyroDataEntry = Shuffleboard.getTab("gyro info")
                                                .add("gyro Yaw Pitch and Roll data", new double[]{0.0,0.0,0.0})
                                                .getEntry();

    //class constuctor
    public Gyro(){
        // Gets the general status VI
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        m_gyro.getGeneralStatus(genStatus);

        // Tare the sensors
        // Sets the yaw,Pitch,Roll and the fuseHeading
        m_gyro.setYaw(0);
        m_gyro.setFusedHeading(0);
    }

    public double[] getYawPitchRoll() {
        double [] yawPitchRoll = new double[3];
        m_gyro.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Send the gyro data to shuffleboard
        m_gyroDataEntry.setDoubleArray(getYawPitchRoll());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    
    }
    
    public Rotation2d getRotation2d() {
        double encoderDegrees = getEncoderDegrees();
        return Rotation2d.fromDegrees(encoderDegrees % 360);
    }

    public void robotInit() {
        // ...
    }
}



