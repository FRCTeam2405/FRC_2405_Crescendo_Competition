// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;
import swervelib.SwerveController;
import swervelib.imu.SwerveIMU;


public class SpeakerAimingDrive extends Command {

  Limelight limelight;
  SwerveContainer swerveDrive;
  SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  Rotation3d rotation3d;
  SwerveIMU imu;
  Pose2d measuredPose;
  Pose2d pose;
  private DoubleSupplier moveX, moveY, turnTheta;
  double lastUpdateTime;
  Rotation2d desiredYaw;

  /** Drive command for aiming at the speaker while moving. */
  public SpeakerAimingDrive(Limelight limelight, SwerveContainer swerveDrive, DoubleSupplier vX, DoubleSupplier vY) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;


    measuredPose = limelight.getMeasuredPose();
    pose = swerveDrive.getPose();

      moveX = vX;
      moveY = vY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, swerveDrive);
  }

  private final TrapezoidProfile.Constraints AIM_PID_CONSTRAINT = new TrapezoidProfile.Constraints(2160.0, 2160.0);

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
    double timestamp = Timer.getFPGATimestamp() - limelight.getLatency();
    imu = swerveDrive.inner.getGyro();
    rotation3d = imu.getRawRotation3d();
    pose = swerveDrive.getPose();
    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedMoveY = Math.pow(moveY.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    
    if (timestamp - lastUpdateTime >= 1) {
     measuredPose = limelight.getMeasuredPose();
     if(limelight.hasTarget() && limelight.tagCount() >= 2) {
       swerveDrive.inner.addVisionMeasurement(new Pose2d(measuredPose.getX(), measuredPose.getY(), measuredPose.getRotation()), timestamp, VecBuilder.fill(0.01, 0.01, 999999999));
       lastUpdateTime = timestamp;
     }
    }
    
    swerveDrive.inner.swerveDrivePoseEstimator.update(swerveDrive.inner.getYaw(), swerveDrive.inner.getModulePositions());

    Optional<Alliance> alliance = DriverStation.getAlliance();

    SmartDashboard.putNumber("poseX", pose.getX());
    SmartDashboard.putNumber("poseY", pose.getY());
    SmartDashboard.putNumber("poseYaw", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("lastUpdateTime", lastUpdateTime);
    SmartDashboard.putNumber("timestamp", timestamp);

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
    desiredYaw = new Rotation2d(offsetX, offsetY);

    SmartDashboard.putNumber("desiredYaw", desiredYaw.getDegrees() % 360);
    SmartDashboard.putNumber("measuredPose", measuredPose.getRotation().getDegrees() % 180);
    
    if (Math.abs(pose.getRotation().getDegrees() - desiredYaw.getDegrees()) > 0.25) {
    ChassisSpeeds chassisSpeeds = swerveDrive.inner.getSwerveController().getTargetSpeeds(
      correctedMoveX, correctedMoveY,
      desiredYaw.getRadians() % (Math.PI * 2),
      (pose.getRotation().getRadians() % (Math.PI * 2)),
      Constants.Swerve.MAX_SPEED
    );
    swerveDrive.inner.drive(chassisSpeeds);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.inner.setHeadingCorrection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled() && Math.abs(pose.getRotation().getDegrees() - desiredYaw.getDegrees()) > 0.25) {
      return true;
    } else {
      return false;
    }
  }
}
