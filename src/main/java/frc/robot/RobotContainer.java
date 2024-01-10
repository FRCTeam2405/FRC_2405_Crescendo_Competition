// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
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
