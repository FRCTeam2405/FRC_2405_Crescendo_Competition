// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveContainer;

public class Turn90Degrees extends Command {

  Pose2d desiredPose;
  SwerveContainer swerveDrive;

  // Creates a new Turn90degrees.
  public Turn90Degrees(SwerveContainer swerveDrive) {
    this.swerveDrive = swerveDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredPose = swerveDrive.getPose().rotateBy(new Rotation2d(Math.PI / 2));
    swerveDrive.inner.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //FIXME! verbose & redundant
    // Try ChassiSpeeds discretize
    ChassisSpeeds desiredSpeeds = swerveDrive.inner.swerveController.getRawTargetSpeeds(
      0, 0,
      desiredPose.getRotation().getRadians(),
      swerveDrive.inner.getPose().getRotation().getRadians()
    );
    swerveDrive.inner.drive(desiredSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.inner.setHeadingCorrection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
