// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;

public class SpeakerAimingDrive extends Command {

  Limelight limelight;
  SwerveContainer swerveDrive;

  /** Drive command for aiming at the speaker while moving. */
  public SpeakerAimingDrive(Limelight limelight, SwerveContainer swerveDrive) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.inner.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get measured pose from the limelight and add it to our pose
    // If the measured pose is null, we cannot detect any Apriltags
    double timestamp = Timer.getFPGATimestamp();
    Pose2d measuredPose = limelight.getMeasuredPose();

    if(measuredPose != null) {
      swerveDrive.inner.addVisionMeasurement(measuredPose, timestamp);
    }

    // Post the pose to dashboard
    Pose2d pose = swerveDrive.inner.getPose();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if(alliance.isEmpty()) {
      return;
    }

    double offsetX;
    double offsetY;

    // Calculate XY offset between robot and speaker,
    // convert to angle, then convert to field-centric angle
    if(alliance.get() == Alliance.Blue) {
      offsetX = Constants.Field.BLUE_SPEAKER_X - pose.getX();
      offsetY = Constants.Field.BLUE_SPEAKER_Y - pose.getY();
    } else {
      offsetX = Constants.Field.RED_SPEAKER_X - pose.getX();
      offsetY = Constants.Field.RED_SPEAKER_Y - pose.getY();
    }

    Rotation2d desiredYaw = new Rotation2d(offsetX, offsetY);

    ChassisSpeeds chassisSpeeds = swerveDrive.inner.getSwerveController().getTargetSpeeds(
      0, 0,
      desiredYaw.getRadians(),
      pose.getRotation().getRadians(),
      Constants.Swerve.MAX_SPEED
    );
    swerveDrive.inner.drive(chassisSpeeds);
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
