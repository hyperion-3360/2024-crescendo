// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
  
  //make a new pigeon
WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(Constants.gyroPort);
 
//dislplay the yaw pitch and roll on the ShuffleBoard
GenericEntry m_gyroDataEntry = Shuffleboard.getTab("gyro info").add("gyro Yaw Pitch and Roll data",
new double[]{0.0,0.0,0.0}).getEntry();

//class constuctor
public Gyro(){
               //gets the general status VI
              PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
              m_gyro.getGeneralStatus(genStatus);

              //sets the yaw,Pitch,Roll and the fuseHeading
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
  m_gyroDataEntry.setDoubleArray(getYawPitchRoll());
  
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
 
  }

  public void robotInit() {
 }
}



