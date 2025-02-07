// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveContainer;

public class TurnToBlueSide extends Command {

  SwerveContainer swerveDrive;

  public TurnToBlueSide(SwerveContainer swervedrive) {
    this.swerveDrive = swervedrive;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swervedrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO! Consider passing in driver input

    // Points towards the side of the field where the amps are located.

    // Requires an accurate field pose - make sure we have at least one
    // vision measurement before driving.
    swerveDrive.driveAbsolute(0, 0, Rotation2d.fromDegrees(-90));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setHeadingCorrection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}