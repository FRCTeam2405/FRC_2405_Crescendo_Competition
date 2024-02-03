// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroGyro extends InstantCommand {

  private SwerveContainer swerve;

  public ZeroGyro(SwerveContainer swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  // Does not seem to run when using an InstantCommand
  @Override
  public void initialize() {
    swerve.zeroGyro();
  }

  @Override
  public void execute() {
    swerve.zeroGyro();
  }
}
