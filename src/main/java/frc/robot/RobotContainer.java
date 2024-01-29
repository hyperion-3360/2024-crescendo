// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Shooter.e_shoot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Sequences;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final DriveTrain m_drive = new DriveTrain();

 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  private final Elevator m_elevator = new Elevator();
  //private final Shooter m_shooter = new Shooter();
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
    
    m_driverController.y()
    .onTrue(
    Sequences.switchToHigh(m_elevator)
    );

    m_driverController.b()
    .onTrue(
    Sequences.switchToIntakeMode(m_elevator)
    );

    m_driverController.a()
    .onTrue(
    Sequences.switchToLow(m_elevator)
    );

    // m_driverController.b()
    // .onTrue(m_shooter.shooting(e_shoot.LOW));
 
    /* wpilib controller example */
    // m_driverController.b().onTrue(m_driveTrain.exampleMethodCommand()); // this would be the
    // shooter button
    // m_coDriverController.rightTrigger().whileTrue(exampleDriveTrainCommand + exampleMath); //this
    // would be the gas button
    // m_coDriverController.
  }
  }

  