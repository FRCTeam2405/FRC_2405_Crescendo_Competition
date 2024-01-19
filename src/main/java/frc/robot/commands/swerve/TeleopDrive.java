// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveContainer;

public class TeleopDrive extends Command {
  private SwerveContainer swerve;

  private DoubleSupplier moveX, moveY, turnTheta;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(SwerveContainer swerveContainer, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotate) {
    swerve = swerveContainer;

    moveX = vX;
    moveY = vY;
    turnTheta = rotate;

    // Required subsystems
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Cube input of XY movement, multiply by max speed
    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedMoveY = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedTurnTheta = turnTheta.getAsDouble() * Constants.Swerve.MAX_ANGULAR_SPEED;

    //TODO! Cleanup, test, & improve
    ChassisSpeeds desiredSpeeds = swerve.inner.swerveController.getRawTargetSpeeds(correctedMoveX, correctedMoveY, correctedTurnTheta);
    swerve.inner.drive(desiredSpeeds);
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
