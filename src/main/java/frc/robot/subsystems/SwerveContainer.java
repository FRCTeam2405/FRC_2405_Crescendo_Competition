// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO! Configure the json files in deploy/swerve

package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveContainer implements Subsystem {

  public static SwerveDrive inner;

  /** Creates a new SwerveContainer. */
  public SwerveContainer() {
    // Set the verbosity before initializing the swerve drive.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // Try to init the swerve drive, and send an error if it fails.
    try {
      inner = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
        .createSwerveDrive(Constants.Swerve.MAX_SPEED);      
    } catch(Exception e) {
      throw new RuntimeException(e);
    }

    

    // Configure PathPlanner and/or Choreo
  }

  @Override
  public void periodic() {

  }
}
