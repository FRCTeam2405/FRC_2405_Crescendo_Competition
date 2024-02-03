// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.Turn90Degrees;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.ZeroGyro;
import frc.robot.controllers.GuitarController;
import frc.robot.subsystems.SwerveContainer;

public class RobotContainer {
  // Initialize drive-team controllers
  //TODO! CommandGenericHID may not be optimal. Decide whether we need to make a helper class or other tool.
  private CommandGenericHID driverController = new CommandGenericHID(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  private GuitarController codriverController = new GuitarController(Constants.Controllers.CODRIVER_CONTROLLER_PORT);

  // Autonomous chooser for SmartDashboard
  private SendableChooser<Command> autonChooser = new SendableChooser<>();

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

    driverController.button(Constants.Controllers.Taranis.ZERO_GYRO_BUTTON).onTrue(new ZeroGyro(swerveDrive));
    driverController.button(Constants.Controllers.Taranis.ROTATE_90_DEGREES).whileTrue(new Turn90Degrees(swerveDrive));
  }

  private DoubleSupplier axisDeadband(CommandGenericHID controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1 : 1;
    return () -> {
      double axisOut = controller.getRawAxis(axis);
      return (Math.abs(axisOut) > deadband) ? axisOut * invertedMultiplier : 0;
    }; 
  }

  // Set up the autonomous routines
  private void configureAutonomous() {
    //TODO! configure all the autons

    // Set a default autonomous to prevent errors
    //TODO! Set this to an autonomous that will still get us points
    autonChooser.setDefaultOption("NONE", Commands.print("No autonomous command selected!"));

    autonChooser.addOption("Square Test", new PathPlannerAuto("Square Test Path"));
    autonChooser.addOption("Square Test Rotate", new PathPlannerAuto("Square Test Path, rotate after drive"));
    autonChooser.addOption("Circle Test", new PathPlannerAuto("Circle Test Path"));
    autonChooser.addOption("Circle Test Rotate", new PathPlannerAuto("Circle Test Path, rotate during drive"));
    autonChooser.addOption("Small Circle Test", new PathPlannerAuto("Small Circle Test Auto"));
    autonChooser.addOption("Small Square Test", new PathPlannerAuto("Small Square Auto"));
    autonChooser.addOption("Backwards", new PathPlannerAuto("Backwards")); 
    autonChooser.addOption("Blue1", new PathPlannerAuto("collect3Blue1"));
    autonChooser.addOption("Small square test rotate", new PathPlannerAuto("Small Rotating Square"));
    autonChooser.addOption("SmallSimpleSquareRotate", new PathPlannerAuto("SmallSimpleSquareRotate"));
    autonChooser.addOption("SmallCircleFacingInwards", new PathPlannerAuto("SmallCircleFacingInwards"));  
    autonChooser.addOption("RotationTest", new PathPlannerAuto("Rotation test"));

    SmartDashboard.putData("autonDropdown", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
