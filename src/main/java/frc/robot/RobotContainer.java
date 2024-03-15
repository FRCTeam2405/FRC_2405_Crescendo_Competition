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
import frc.robot.commands.DirectDriveArm;
import frc.robot.commands.GetVisionMeasurement;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.SetStartPose;
import frc.robot.commands.SetStartPose.StartPosition;
import frc.classes.AutonChooser;
import frc.robot.commands.shooting.FireWhenReadyVelocity;
import frc.robot.commands.shooting.IntakeNote;
import frc.robot.commands.shooting.IntakeOnly;
import frc.robot.commands.shooting.PrimeShooter;
import frc.robot.commands.swerve.SpeakerAimingDrive;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.ZeroGyro;
import frc.robot.controllers.GuitarController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;
import frc.robot.subsystems.shooting.Feeder;
import frc.robot.subsystems.shooting.Intake;
import frc.robot.subsystems.shooting.Shooter;

public class RobotContainer {
  // Initialize drive-team controllers
  private CommandGenericHID driverController = new CommandGenericHID(Constants.Controllers.DRIVER_CONTROLLER_PORT);
  private GuitarController codriverController = new GuitarController(Constants.Controllers.CODRIVER_CONTROLLER_PORT);

  private SendableChooser<Command> testAutonChooser = new SendableChooser<>();

  // Initialize subsystems
  private SwerveContainer swerveDrive = new SwerveContainer();
  private Limelight limelight = new Limelight();

  // Comp bot only
  private LEDLights sysLighting = new LEDLights(Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT, Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
  private Intake sysIntake = new Intake();
  private Feeder sysFeeder = new Feeder();
  private Shooter sysShooter = new Shooter();
  private Arm sysArm = new Arm();
  private Dashboard sysDashboard = new Dashboard(sysShooter, sysFeeder, sysIntake, sysArm);

  // Initialization code for our robot
  public RobotContainer() {
    if(DriverStation.getAlliance().isPresent()) {
      SwerveContainer.allianceColor = DriverStation.getAlliance().get();
    } else {
      SwerveContainer.allianceColor = null;
    }

    configureBindings();
    configureAutonomous();
    // SmartDashboard.putNumber(Constants.Dashboard.Utility.Widgets.ROBOT_EMOTION_SETTING_NAME, codriverController.getRawAxis(Constants.Controllers.Guitar.ROBOT_EMOTION_ID));
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
    driverController.button(Constants.Controllers.Taranis.ADD_VISION_MEASURMENT_BUTTON).whileTrue(new GetVisionMeasurement(swerveDrive, limelight));
    driverController.button(Constants.Controllers.Taranis.ROTATE_TO_SPEAKER_BUTTON).whileTrue(new SpeakerAimingDrive(limelight, swerveDrive, 
     axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_X_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true), 
     axisDeadband(driverController, Constants.Controllers.Taranis.DRIVE_Y_AXIS, Constants.Controllers.Taranis.DRIVE_DEADBAND, true)
    ));


    // Comp bot only

    // intake commands
    
    if (sysArm.getArmPosition() <= Constants.Arm.SetPoints.HOME + 10) {
      driverController.button(
        Constants.Controllers.Taranis.INTAKE_NOTE_BUTTON)
        .whileTrue(new IntakeNote(sysIntake, sysFeeder, sysLighting, sysDashboard));
    }
    else {
      sysLighting.setColorOne(Constants.LEDs.LED_ACTIONS.INTAKE_INVALID);
      sysLighting.setColorTwo(Constants.LEDs.LED_ACTIONS.INTAKE_INVALID);
    }

    driverController.button(
      Constants.Controllers.Taranis.REVERSE_INTAKE_NOTE_BUTTON)
      .whileTrue(new IntakeNote(sysIntake, sysFeeder, sysLighting, sysDashboard, 
      () -> Constants.Intake.Motors.RIGHT_INTAKE_REVERSE_SPEED_MAX, 
      () -> Constants.Feeder.Motors.REVERSE_FEEDER_INTAKING_SPEED));



    // shooter command

    if (sysArm.getArmPosition() >= Constants.Arm.SetPoints.AMP - 10) {
       codriverController.pov(
        Constants.Controllers.Guitar.STRUM_DOWN)
        .whileTrue(new FireWhenReadyVelocity(sysShooter, sysFeeder, sysLighting, sysDashboard,
        () -> Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_AMP, 
        () -> Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_AMP,
        () -> Constants.Feeder.Motors.TOP_FEEDER_SHOOTING_SPEED,
        () -> Constants.Feeder.Motors.BOTTOM_FEEDER_SHOOTING_SPEED));

        codriverController.pov(
        Constants.Controllers.Guitar.STRUM_UP)
        .onTrue(new PrimeShooter(sysShooter, sysLighting, 
                  () -> Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_AMP, 
                  () -> Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_AMP));
    }
    else {
      codriverController.pov(
        Constants.Controllers.Guitar.STRUM_DOWN)
        .whileTrue(new FireWhenReadyVelocity(sysShooter, sysFeeder, sysLighting, sysDashboard,
                   () -> Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_DEFAULT, 
                   () -> Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_DEFAULT,
                   () -> Constants.Feeder.Motors.TOP_FEEDER_SHOOTING_SPEED,
                   () -> Constants.Feeder.Motors.BOTTOM_FEEDER_SHOOTING_SPEED));

      codriverController.pov(
        Constants.Controllers.Guitar.STRUM_UP)
        .onTrue(new PrimeShooter(sysShooter, sysLighting, 
              () -> Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_DEFAULT, 
              () -> Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_DEFAULT));
    }
    
    // arm commands
    codriverController.button(Constants.Controllers.Guitar.RED_FRET)
                        .onTrue(new MoveArmToPosition(sysArm, sysDashboard, () -> Constants.Arm.SetPoints.AMP));
    codriverController.button(Constants.Controllers.Guitar.GREEN_FRET)
                        .onTrue(new MoveArmToPosition(sysArm, sysDashboard, () -> Constants.Arm.SetPoints.HOME));
    codriverController.button(Constants.Controllers.Guitar.BLUE_FRET)
                        .onTrue(new MoveArmToPosition(sysArm, sysDashboard));
    codriverController.button(Constants.Controllers.Guitar.ORANGE_FRET)
     .onTrue(new MoveArmToPosition(sysArm, sysDashboard, () -> Constants.Arm.SetPoints.CLIMB));
  }

  private DoubleSupplier axisDeadband(CommandGenericHID controller, int axis, double deadband, boolean inverted) {
    double invertedMultiplier = inverted ? -1 : 1;
    return () -> {
      double axisOut = controller.getRawAxis(axis);
      return (Math.abs(axisOut) > deadband) ? axisOut * invertedMultiplier : 0;
    }; 
  }

  DoubleSupplier sup = () -> (0);

  // Set up the autonomous routines
  private void configureAutonomous() {
    // Register named commands for pathplanner
    // This must be done before initializing autos
    NamedCommands.registerCommand("GetVisionMeasurement", new GetVisionMeasurement(swerveDrive, limelight));
    NamedCommands.registerCommand("RotateToSpeaker", new SpeakerAimingDrive(limelight, swerveDrive, sup, sup));
    NamedCommands.registerCommand("SetBlue1", new SetStartPose(swerveDrive, StartPosition.Blue1));
    NamedCommands.registerCommand("SetBlue2", new SetStartPose(swerveDrive, StartPosition.Blue2));
    NamedCommands.registerCommand("SetBlue3", new SetStartPose(swerveDrive, StartPosition.Blue3));
    NamedCommands.registerCommand("SetRed1", new SetStartPose(swerveDrive, StartPosition.Red1));
    NamedCommands.registerCommand("SetRed2", new SetStartPose(swerveDrive, StartPosition.Red2));
    NamedCommands.registerCommand("SetRed3", new SetStartPose(swerveDrive, StartPosition.Red3));

    // Comp bot only
    NamedCommands.registerCommand("Shoot", new FireWhenReadyVelocity(sysShooter, sysFeeder, sysLighting, sysDashboard));
    NamedCommands.registerCommand("Intake", new IntakeNote(sysIntake, sysFeeder, sysLighting, sysDashboard));

    // Set a default autonomous to prevent errors
    testAutonChooser.setDefaultOption("NONE", Commands.print("No autonomous command selected!"));

    testAutonChooser.addOption("Small Circle Test", new PathPlannerAuto("Small Circle Test Auto"));
    testAutonChooser.addOption("Small Square Test", new PathPlannerAuto("Small Square Auto"));
    testAutonChooser.addOption("SmallCircleFacingInwards", new PathPlannerAuto("SmallCircleFacingInwards"));  
    testAutonChooser.addOption("RotationTest", new PathPlannerAuto("Rotation test"));
    testAutonChooser.addOption("Right Turn", new PathPlannerAuto("Left Turn"));
    testAutonChooser.addOption("Right Contained Turn", new PathPlannerAuto("rightTurn"));

    SmartDashboard.putData("testAutonDropdown", testAutonChooser);
  }

  public Command getAutonomousCommand() {
   if (sysDashboard.getAutonChooser().getSelected() != null && testAutonChooser.getSelected() != null) {
    Commands.print("2 autonomous commands selected");
    return null;
   } else {
    if (testAutonChooser.getSelected() != null) {
     return testAutonChooser.getSelected();
    } else {
     if (sysDashboard.getAutonChooser().getSelected() != null) {
      return sysDashboard.getAutonChooser().getSelected();
     } else {
      Commands.print("No autonomous commands selected");
      return null;
     }
    }
   }
  }
}
