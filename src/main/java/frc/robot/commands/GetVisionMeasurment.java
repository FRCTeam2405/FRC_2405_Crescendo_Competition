// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dashboard;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;

public class GetVisionMeasurment extends Command {
  private SwerveContainer swerve;
  private Limelight limelight;
  double lastUpdateTime = 0;
  double timestamp;

  /** Creates a new GetVisionMeasurment. */
  public GetVisionMeasurment(SwerveContainer swerve, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;

    addRequirements(limelight, swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.initialize();
    lastUpdateTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Post the pose to dashboard
    Pose2d pose = swerve.getPose();

    // Vision measurement
    timestamp = Timer.getFPGATimestamp();
    Pose2d measuredPose = limelight.getMeasuredPose();
    if(limelight.hasTarget() && limelight.tagCount() >= 2 && timestamp - lastUpdateTime >= 1) {
      swerve.inner.addVisionMeasurement(new Pose2d(measuredPose.getX(), measuredPose.getY(), pose.getRotation()), timestamp/**, visionMeasurmentStdDevs*/);
      lastUpdateTime = timestamp;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled() && (timestamp > 0.25 | (limelight.hasTarget() && limelight.tagCount() >= 2 && timestamp - lastUpdateTime >= 1))) {
     return true;
    } else {
     return false;
    }
  }
}
