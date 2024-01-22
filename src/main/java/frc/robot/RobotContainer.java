// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.controllers.GuitarController;
import frc.robot.subsystems.SwerveContainer;

public class RobotContainer {
  // Initialize drive-team controllers
  //TODO! CommandGenericHID may not be optimal. Decide whether we need to make a helper class or other tool.
  private CommandGenericHID driverController = new CommandGenericHID(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  private GuitarController codriverController = new GuitarController(Constants.Controllers.CODRIVER_CONTROLLER_PORT);

  // Initialize subsystems
  private SwerveContainer swerveDrive = new SwerveContainer();

  // Initialization code for our robot
  public RobotContainer() {
    configureBindings();
    configureAutonomous();
  }

  // Set up the driver controls
  private void configureBindings() {
    // Temporary controls for testing
    //TODO! Competition controls
    swerveDrive.setDefaultCommand(new TeleopDrive(swerveDrive,
      // Invert X Axis - WPIlib is forward-positive, joystick is down-positive
      axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_X_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true),
      // Invert Y Axis - WPILib is left-positive, joystick is right-positive
      axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_Y_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true),
      axisDeadband(driverController, Constants.Controllers.Taranis.ROTATE_AXIS, Constants.Controllers.Taranis.ROTATE_DEADBAND, false)
    ));
  }

  private DoubleSupplier axisDeadband(CommandGenericHID controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1 : 1;
    return () -> {
      double axisOut = controller.getRawAxis(axis);
      return (Math.abs(axisOut) > deadband) ? axisOut * invertedMultiplier : 0;
    }; 
  }

  // Set up the autonomous routines
  private void configureAutonomous() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
