// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveContainer;
import swervelib.imu.SwerveIMU;

public class TeleopDrive extends Command {
  private SwerveContainer swerveDrive;
  Optional<Alliance> alliance;
  Rotation3d rotation3d;
  SwerveIMU imu;

  private DoubleSupplier moveX, moveY, turnTheta;

  /** Drive command for typical teleop movement. */
  public TeleopDrive(SwerveContainer swerveContainer, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotate) {
    swerveDrive = swerveContainer;

    moveX = vX;
    moveY = vY;
    turnTheta = rotate;
    
    // Required subsystems
    addRequirements(swerveDrive);
  }

  /*
  Standard deviation for apriltag position setting
  private Matrix<N3, N1> visionMeasurmentStdDevs = VecBuilder.fill(0.01, 0.01, 0.01);
  */
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the motors to coast
    swerveDrive.setMotorBrake(false);
    alliance = DriverStation.getAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Cube input of XY movement, multiply by max speed
    double correctedMoveX = moveX.getAsDouble() * Constants.Swerve.MAX_SPEED;
    double correctedMoveY = moveY.getAsDouble() * Constants.Swerve.MAX_SPEED;
    double correctedTurnTheta = turnTheta.getAsDouble() * Constants.Swerve.MAX_ANGULAR_SPEED;

    // Invert inputs if we're on the red side of the field
    // so that movement is still relative to driver
    // Requires at least one vision measurement to be accurate
    //TODO! test this
    // if(alliance.get() == Alliance.Red) {
    //   correctedMoveX *= -1;
    //   correctedMoveY *= -1;
    // }

    swerveDrive.driveRelative(correctedMoveX, correctedMoveY, correctedTurnTheta);
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
