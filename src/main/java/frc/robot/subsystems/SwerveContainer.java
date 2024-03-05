// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO! Configure the json files in deploy/swerve

package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveContainer implements Subsystem {

  private SwerveDrive inner;

  //TODO! put these elsewhere
  public static int robotTeamNumber = HALUtil.getTeamNumber();
  public static Alliance allianceColor;

  /** Creates a new SwerveContainer. */
  public SwerveContainer() {
    // Set the verbosity before initializing the swerve drive.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // TODO! Prefer removing test config before competition
    File swerveConfig = 
      // If the team number is 9998, use the testbot config
      // Otherwise, use the competition config
      robotTeamNumber == 9998
        ? new File(Filesystem.getDeployDirectory(), "swerve/test")
        : new File(Filesystem.getDeployDirectory(), "swerve/competition");

    // Try to init the swerve drive, and send an error if it fails.
    try {
      inner = new SwerveParser(swerveConfig)
        .createSwerveDrive(Constants.Swerve.MAX_SPEED);      
    } catch(Exception e) {
      throw new RuntimeException(e);
    }

    // Configure PathPlanner and/or Choreo
    configurePathplanner();
  }

  private void configurePathplanner() {
    AutoBuilder.configureHolonomic(
      // Supply methods for Pathplanner to use to control the robot
      this::getPose,
      this::resetOdometry,
      this::getRobotVelocity,
      this::setChassisSpeeds,
      // PID and speed constants
      Constants.Swerve.PATH_PLANNER_CONFIG,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This should flip the path being followed to the red side of the field.
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      },
      this
    );
  }

  @Override
  public void periodic() {
    // Put the measured team number to the dashboard for diagnostics
    SmartDashboard.putNumber("robotTeamNumber", robotTeamNumber);
  }
  
  // Pass-through functions
  // These just pass the parameters and returns
  // to the inner swerve drive class

  // Getters
  public Pose2d getPose() { return inner.getPose(); }
  public ChassisSpeeds getRobotVelocity() { return inner.getRobotVelocity(); }
  public Rotation2d getYaw() { return inner.getYaw(); }

  // Setters
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) { inner.setChassisSpeeds(chassisSpeeds); }

  /** Sets whether the driver motors will brake.
   * Braking causes stress on the motors, but
   * prevents slippage. */
  public void setMotorBrake(boolean brakeEnabled) {
    inner.setMotorIdleMode(brakeEnabled);
  }

  /** Drive the robot using relative rotation (turning left and right).
   *  Do not use for autonomous driving.
   * 
   * @param moveX X velocity in meters per second
   * @param moveY Y velocity in meters per second
   * @param turnTheta Angular velocity in radians per second
   */
  public void driveRelative(double moveX, double moveY, double turnTheta) {
    ChassisSpeeds desiredSpeeds = inner.swerveController.getRawTargetSpeeds(moveX, moveY, turnTheta);
    inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);
  }

  /** Drive the robot using absolute rotation (point towards a direction).
   * Use for assisted or autonomous driving.
   * 
   * @param moveX X velocity in meters per second
   * @param moveY Y velocity in meters per second
   * @param rotation2d Desired rotation
   */
  public void driveAbsolute(double moveX, double moveY, Rotation2d rotation2d) {
    ChassisSpeeds desiredSpeeds = inner.swerveController.getRawTargetSpeeds(
      moveX,
      moveY,
      rotation2d.getRadians(),
      getYaw().getRadians()
    );
    inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);
  }

  //TODO! Document
  public void setHeadingCorrection(boolean correctionEnabled) {
    inner.setHeadingCorrection(correctionEnabled);
  }

  /** Update the pose estimator without a vision reading. */
  public void updatePose() {
    inner.swerveDrivePoseEstimator.update(inner.getYaw(), inner.getModulePositions());
  }

  public void resetOdometry(Pose2d pose) {
    inner.resetOdometry(pose);
  }

  public void zeroGyro() {
    inner.zeroGyro();
  }
}
