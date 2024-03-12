// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.SwerveContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetStartPose extends InstantCommand {

  public enum StartPosition {
    Blue1,
    Blue2,
    Blue3,
    Red1,
    Red2,
    Red3
  }

  SwerveContainer swerveDrive;
  StartPosition startPosition;

  public SetStartPose(SwerveContainer swerveDrive, StartPosition startPosition) {
    Objects.requireNonNull(startPosition);
    this.startPosition = startPosition;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double startX;
    double startY;
    double startRotation;

    // Choose start position based on autonomous input
    switch (startPosition) {
      case Blue1:
        startX = Constants.Auton.startPose.BLUE_1_X;
        startY = Constants.Auton.startPose.BLUE_1_Y;
        startRotation = Constants.Auton.startPose.BLUE_1_ROTATION;
        break;
      
      case Blue2:
        startX = Constants.Auton.startPose.BLUE_2_X;
        startY = Constants.Auton.startPose.BLUE_2_Y;
        startRotation = Constants.Auton.startPose.BLUE_1_ROTATION;
        break;
      
      case Blue3:
        startX = Constants.Auton.startPose.BLUE_3_X;
        startY = Constants.Auton.startPose.BLUE_3_Y;
        startRotation = Constants.Auton.startPose.BLUE_1_ROTATION;
        break;
      
      case Red1:
        // TODO! make red positions
        // startX = Constants.Auton.startPose.RED_1_X;
        // startY = Constants.Auton.startPose.RED_1_Y;
        // startRotation = Constants.Auton.startPose.RED_1_ROTATION;
        // break;
        return;
      
      case Red2:
        // startX = Constants.Auton.startPose.RED_2_X;
        // startY = Constants.Auton.startPose.RED_2_Y;
        // startRotation = Constants.Auton.startPose.RED_2_ROTATION;
        // break;
        return;

      case Red3:
        // startX = Constants.Auton.startPose.RED_3_X;
        // startY = Constants.Auton.startPose.RED_3_Y;
        // startRotation = Constants.Auton.startPose.RED_3_ROTATION;
        // break;
        return;
      
      default:
        // unreachable
        return;
    }

    Rotation2d rotation = new Rotation2d(startRotation);
    Pose2d pose = new Pose2d(startX, startY, rotation);

    // reset pose with our initial auton position
    swerveDrive.resetPose(rotation, swerveDrive.getModulePositions(), pose);
  }
}
