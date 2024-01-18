// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.controllers.GuitarController;

public class RobotContainer {
  // Initialize drive-team controllers
  //TODO! CommandGenericHID may not be optimal. Decide whether we need to make a helper class or other tool.
  private CommandGenericHID driverController = new CommandGenericHID(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  private GuitarController codriverController = new GuitarController(Constants.Controllers.CODRIVER_CONTROLLER_PORT);

  // Initialization code for our robot
  public RobotContainer() {
    configureBindings();
    configureAutonomous();
  }

  // Set up the driver controls
  private void configureBindings() {}
  // Set up the autonomous routines
  private void configureAutonomous() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
