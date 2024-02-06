// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveContainer;
import frc.robot.subsystems.Limelight;

public class RotateToApriltag extends Command {

  double desiredYaw;
  SwerveContainer swerveDrive;
  Limelight limelight;

  /** Creates a new RotateToApriltag. */
  public RotateToApriltag(SwerveContainer swerveDrive, Limelight limelight) {
  this.swerveDrive = swerveDrive;
  this.limelight = limelight;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.inner.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredYaw = limelight.getTargetPose(7)[5];

        ChassisSpeeds desiredSpeeds = swerveDrive.inner.swerveController.getRawTargetSpeeds(
      0, 0,
      desiredYaw*Math.PI/180,
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
