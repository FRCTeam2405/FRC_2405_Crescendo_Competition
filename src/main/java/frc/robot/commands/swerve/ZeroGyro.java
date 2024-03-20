// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroGyro extends InstantCommand {

  private SwerveContainer swerveDrive;

  public ZeroGyro(SwerveContainer swerveDrive) {
    this.swerveDrive = swerveDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  // Does not seem to run when using an InstantCommand
  @Override
  public void initialize() {
    swerveDrive.zeroGyro();
  }

  @Override
  public void execute() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    Pose2d pose = swerveDrive.getPose();
    Rotation2d zeroedRotation = new Rotation2d(Math.PI);
    Pose2d zeroedRobotPosition = new Pose2d(pose.getX(), pose.getY(), zeroedRotation);

    if(alliance.get() == Alliance.Blue) {
      swerveDrive.zeroGyro();
    } else {
      swerveDrive.resetPose(swerveDrive.getYaw(), swerveDrive.getModulePositions(), zeroedRobotPosition);
    }
  }
}
