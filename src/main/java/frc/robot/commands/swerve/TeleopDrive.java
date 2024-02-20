// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import swervelib.imu.SwerveIMU;

public class TeleopDrive extends Command {
  private SwerveContainer swerve;
  private Limelight limelight;
  Rotation3d rotation3d;
  SwerveIMU imu;
  double lastUpdateTime = 0;

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

  double yawCorrection;

  /** 
  // Standard deviation for apriltag position setting
  private Matrix<N3, N1> visionMeasurmentStdDevs = VecBuilder.fill(0.01, 0.01, 0.01);
  */
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the motors to coast
    swerve.inner.setMotorIdleMode(false);
    limelight.initialize();
    lastUpdateTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Post the pose to dashboard
    Pose2d pose = swerve.getPose();

    imu = swerve.inner.getGyro();
    rotation3d = imu.getRawRotation3d();

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
    if(limelight.hasTarget() && limelight.tagCount() >= 2 && timestamp - lastUpdateTime >= 1000) {
      swerve.inner.addVisionMeasurement(new Pose2d(measuredPose.getX(), measuredPose.getY(), pose.getRotation()), timestamp/**, visionMeasurmentStdDevs*/);
      yawCorrection = measuredPose.getRotation().getRadians() - pose.getRotation().getRadians();
      lastUpdateTime = timestamp;
    }

    // puts yawCorrected on the dashboard
    SmartDashboard.putNumber("yawCorrected", (pose.getRotation().getDegrees() + Math.toDegrees(yawCorrection)) % 360);
    SmartDashboard.putNumber("measuredPose", measuredPose.getRotation().getDegrees() % 180);

    SmartDashboard.putNumber("lastUpdateTime", lastUpdateTime);

    swerve.inner.swerveDrivePoseEstimator.update(swerve.inner.getYaw(), swerve.inner.getModulePositions());
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
