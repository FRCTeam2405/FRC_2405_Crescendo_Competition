// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;
import swervelib.SwerveController;

public class TeleopDrive extends Command {
  private SwerveContainer swerve;
  private Limelight limelight;

  private DoubleSupplier moveX, moveY, turnTheta;

  /** Drive command for typical teleop movement. */
  public TeleopDrive(SwerveContainer swerveContainer, Limelight limelight, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotate) {
    swerve = swerveContainer;
    this.limelight = limelight;

    moveX = vX;
    moveY = vY;
    turnTheta = rotate;

    // Required subsystems
    addRequirements(swerve, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the motors to coast
    swerve.inner.setMotorIdleMode(false);
    limelight.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Post the pose to dashboard
    Pose2d pose = swerve.getPose();

    SmartDashboard.putNumber("poseX", pose.getX());
    SmartDashboard.putNumber("poseY", pose.getY());
    SmartDashboard.putNumber("poseYaw", pose.getRotation().getDegrees());

    // Cube input of XY movement, multiply by max speed
    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedMoveY = Math.pow(moveY.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedTurnTheta = turnTheta.getAsDouble() * Constants.Swerve.MAX_ANGULAR_SPEED;

    //TODO! Cleanup, test, & improve
    ChassisSpeeds desiredSpeeds = swerve.inner.swerveController.getRawTargetSpeeds(correctedMoveX, correctedMoveY, correctedTurnTheta);
    swerve.inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);

    // Vision measurement
    double timestamp = Timer.getFPGATimestamp();
    Pose2d measuredPose = limelight.getMeasuredPose();

    if(measuredPose != null && measuredPose.getX() != 0) {
      swerve.inner.addVisionMeasurement(measuredPose, timestamp);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
