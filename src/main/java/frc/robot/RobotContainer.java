// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.classes.AutonChooser;
import frc.robot.commands.shooting.FireWhenReadyVelocity;
import frc.robot.commands.shooting.IntakeNote;
import frc.robot.commands.shooting.IntakeOnly;
import frc.robot.commands.swerve.RotateToApriltag;
import frc.robot.commands.swerve.SpeakerAimingDrive;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.Turn90Degrees;
import frc.robot.commands.swerve.ZeroGyro;
import frc.robot.controllers.GuitarController;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;
import frc.robot.subsystems.shooting.Feeder;
import frc.robot.subsystems.shooting.Intake;
import frc.robot.subsystems.shooting.Shooter;

public class RobotContainer {
  // Initialize drive-team controllers
  //TODO! CommandGenericHID may not be optimal. Decide whether we need to make a helper class or other tool.
  private CommandGenericHID driverController = new CommandGenericHID(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  private GuitarController codriverController = new GuitarController(Constants.Controllers.CODRIVER_CONTROLLER_PORT);

  // Autonomous chooser for SmartDashboard
  private SendableChooser<Command> testAutonChooser = new SendableChooser<>();
  private SendableChooser<String> startPoseChooser = new SendableChooser<>();
  private SendableChooser<String> firstNoteChooser = new SendableChooser<>();

  // Initialize subsystems
  private SwerveContainer swerveDrive = new SwerveContainer();
  private Limelight limelight = new Limelight();
  // Below systems only on competition bot
  // private Intake intake = new Intake();
  //TODO! enable when shooter and feeder are ready
  // private Feeder feeder = new Feeder();
  // private Shooter shooter = new Shooter();

  // Initialization code for our robot
  public RobotContainer() {
    if(DriverStation.getAlliance().isPresent()) {
      SwerveContainer.allianceColor = DriverStation.getAlliance().get();
    } else {
      SwerveContainer.allianceColor = null;
    }

    configureBindings();
    configureAutonomous();
  }

  // Set up the driver controls
  private void configureBindings() {
    // Temporary controls for testing
    //TODO! Competition controls
    swerveDrive.setDefaultCommand(new TeleopDrive(swerveDrive, limelight,
      // Invert X Axis - WPIlib is forward-positive, joystick is down-positive
      axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_X_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true),
      // Invert Y Axis - WPILib is left-positive, joystick is right-positive
      axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_Y_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true),
      axisDeadband(driverController, Constants.Controllers.Taranis.ROTATE_AXIS, Constants.Controllers.Taranis.ROTATE_DEADBAND, false)
    ));

    driverController.button(Constants.Controllers.Taranis.ZERO_GYRO_BUTTON).onTrue(new ZeroGyro(swerveDrive));
    driverController.button(Constants.Controllers.Taranis.ROTATE_90_DEGREES_BUTTON).whileTrue(new Turn90Degrees(swerveDrive));
    driverController.button(Constants.Controllers.Taranis.ROTATE_TO_APRILTAG_BUTTON).whileTrue(new SpeakerAimingDrive(limelight, swerveDrive, 
     axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_X_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true), 
     axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_Y_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true)
    ));

    //TODO! switch intake only with intake note when feeder is available
    // driverController.button(
    //   Constants.Controllers.Taranis.INTAKE_NOTE_BUTTON)
    //   .whileTrue(new IntakeNote(intake, feeder));
    // driverController.button(
    //   Constants.Controllers.Taranis.INTAKE_NOTE_BUTTON)
    //   .whileTrue(new IntakeOnly(intake));
    //TODO! enable when shooter and feeder are ready
    // driverController.button(
    //   Constants.Controllers.Taranis.FIRE_WHEN_READY_BUTTON)
    //   .whileTrue(new FireWhenReadyVelocity(shooter, feeder));
  }

  private DoubleSupplier axisDeadband(CommandGenericHID controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1 : 1;
    return () -> {
      double axisOut = controller.getRawAxis(axis);
      return (Math.abs(axisOut) > deadband) ? axisOut * invertedMultiplier : 0;
    }; 
  }

  private ShuffleboardTab dashboardTabAuto;
  private ComplexWidget dashboardEntryRobotStartPose;
  private ComplexWidget dashboardEntryFirstNote;

  // Set up the autonomous routines
  private void configureAutonomous() {
    // Register named commands for pathplanner
    // This must be done before initializing autos
    NamedCommands.registerCommand("Turn90Degrees", new Turn90Degrees(swerveDrive));

    // Set a default autonomous to prevent errors
    //TODO! Consider setting this to an autonomous that will still get us points
    testAutonChooser.setDefaultOption("NONE", Commands.print("No autonomous command selected!"));


    testAutonChooser.addOption("Small Circle Test", new PathPlannerAuto("Small Circle Test Auto"));
    testAutonChooser.addOption("Small Square Test", new PathPlannerAuto("Small Square Auto"));
    testAutonChooser.addOption("SmallCircleFacingInwards", new PathPlannerAuto("SmallCircleFacingInwards"));  
    testAutonChooser.addOption("RotationTest", new PathPlannerAuto("Rotation test"));

    startPoseChooser.addOption("blue1", "blue1");
    startPoseChooser.addOption("blue2", "blue2");
    startPoseChooser.addOption("blue3", "blue3");
    startPoseChooser.addOption("red1", "red1");
    startPoseChooser.addOption("red2", "red2");
    startPoseChooser.addOption("red3", "red3");

    dashboardTabAuto = Shuffleboard.getTab(("Auto"));
    dashboardEntryRobotStartPose = dashboardTabAuto.add(
      "Robots starting position", startPoseChooser)
     .withWidget(BuiltInWidgets.kComboBoxChooser)
     .withPosition(0, 0)
     .withSize(2, 1);

    firstNoteChooser.addOption("note1", "note1");
    firstNoteChooser.addOption("note2", "note2");
    firstNoteChooser.addOption("note3", "note3");
    firstNoteChooser.addOption("note4", "note4");
    firstNoteChooser.addOption("note8", "note8");

    dashboardTabAuto = Shuffleboard.getTab(("Auto"));
    dashboardEntryFirstNote = dashboardTabAuto.add(
      "First Note In Auto", startPoseChooser)
     .withWidget(BuiltInWidgets.kComboBoxChooser)
     .withPosition(0, 1)
     .withSize(2, 1);

    SmartDashboard.putData("autonDropdown", testAutonChooser);
  }

  public Command getAutonomousCommand() {
    return testAutonChooser.getSelected();
  }
}
