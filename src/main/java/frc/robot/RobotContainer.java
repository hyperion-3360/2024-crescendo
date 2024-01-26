// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WCPSwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //private final DriveTrain m_drive = new DriveTrain();
  private final Elevator m_elevator = new Elevator();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
       new CommandXboxController(OperatorConstants.kDriverControllerPort);

       private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    System.out.print("CONTA>INER");

    // m_drive.setDefaultCommand(
    //     m_drive.driveCommand(
    //         () -> {
    //           final var driverY = m_driverController.getLeftY();
    //           return driverY;
    //         },
    //         () -> {
    //           final var driverX = m_driverController.getLeftX();
    //           return driverX;
    //         },
    //         () -> -m_driverController.getRightX(),
    //         false));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.rightTrigger().
    whileTrue(m_elevator.setElevatorSpeed(0.1))
    .onFalse(m_elevator.stop());

    m_driverController.leftTrigger().
    whileTrue(m_elevator.setElevatorSpeed(-0.1))
    .onFalse(m_elevator.stop());

    m_driverController.a()
    .toggleOnTrue(m_elevator.setElevator());

        m_driverController.b()
    .toggleOnTrue(m_elevator.setElevator());

        m_driverController.x()
    .toggleOnTrue(m_elevator.setElevator());

    /* wpilib controller example */
    // m_driverController.b().onTrue(m_driveTrain.exampleMethodCommand()); // this would be the shooter button
    // m_coDriverController.rightTrigger().whileTrue(exampleDriveTrainCommand + exampleMath); //this would be the gas button
    // m_coDriverController.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    /* how to get autonomous commands */
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
