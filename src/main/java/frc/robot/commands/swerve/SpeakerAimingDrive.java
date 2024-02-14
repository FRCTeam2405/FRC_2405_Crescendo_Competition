// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;
import swervelib.imu.SwerveIMU;


public class SpeakerAimingDrive extends Command {

  Limelight limelight;
  SwerveContainer swerveDrive;
  SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  Rotation3d rotation3d;
  SwerveIMU imu;

  /** Drive command for aiming at the speaker while moving. */
  public SpeakerAimingDrive(Limelight limelight, SwerveContainer swerveDrive) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, swerveDrive);
  }
  /**
  // Standard deviation for apriltag position setting
  private Matrix<N3, N1> visionMeasurmentStdDevs = VecBuilder.fill(0.01, 0.01, 0.01);
  */
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
    double timestamp = Timer.getFPGATimestamp() - limelight.getLatency();
    Pose2d measuredPose = limelight.getMeasuredPose();
    imu = swerveDrive.inner.getGyro();
    rotation3d = imu.getRawRotation3d();
    
    if(limelight.hasTarget() && limelight.tagCount() >= 2) {
      swerveDrive.inner.addVisionMeasurement(measuredPose, timestamp/**, visionMeasurmentStdDevs*/);
    }
    
    // swerveDrive.inner.swerveDrivePoseEstimator.update(swerveDrive.inner.getYaw(), swerveDrive.inner.getModulePositions());

    // Post the pose to dashboard
    Pose2d pose = swerveDrive.getPose();
    Optional<Alliance> alliance = DriverStation.getAlliance();

    SmartDashboard.putNumber("poseX", pose.getX());
    SmartDashboard.putNumber("poseY", pose.getY());
    SmartDashboard.putNumber("poseYaw", pose.getRotation().getDegrees());

    if(alliance.isEmpty()) {
      return;
    }

    // Relative positions from the robot to the speaker.
    double offsetX;
    double offsetY;
    double offsetZ;

    // Calculate XY offset between robot and speaker,
    // convert to angle, then convert to field-centric angle
    if(alliance.get() == Alliance.Blue) {
      offsetX = Constants.Field.BLUE_SPEAKER_X - pose.getX();
      offsetY = Constants.Field.BLUE_SPEAKER_Y - pose.getY();
      offsetZ = Constants.Field.BLUE_SPEAKER_Z - Constants.Shooter.SHOOTER_HEIGHT;
    } else {
      offsetX = Constants.Field.RED_SPEAKER_X - pose.getX();
      offsetY = Constants.Field.RED_SPEAKER_Y - pose.getY();
      offsetZ = Constants.Field.RED_SPEAKER_Z - Constants.Shooter.SHOOTER_HEIGHT;
    }

    SmartDashboard.putNumber("offsetX", offsetX);
    SmartDashboard.putNumber("offsetY", offsetY);
    SmartDashboard.putNumber("offsetZ", offsetZ);

    // calculate direct distance and ground distance to the speaker
    double floorDistance = Math.hypot(offsetX, offsetY);
    double directDistance = Math.hypot(floorDistance, offsetZ);

    // calculate pitch and yaw from the shooter to the speaker
    Rotation2d desiredPitch = new Rotation2d(floorDistance, offsetZ);
    Rotation2d desiredYaw = new Rotation2d(offsetX, offsetY);

    

    SmartDashboard.putNumber("desiredYaw", desiredYaw.getDegrees() % 360);

    ChassisSpeeds chassisSpeeds = swerveDrive.inner.getSwerveController().getTargetSpeeds(
      0, 0,
      desiredYaw.getRadians() % (Math.PI * 2),
      pose.getRotation().getRadians() % (Math.PI * 2),
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
