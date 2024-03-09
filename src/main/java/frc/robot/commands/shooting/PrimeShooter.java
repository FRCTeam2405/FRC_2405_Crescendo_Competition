// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooting.Shooter;

public class PrimeShooter extends Command {

  private final Shooter sysShooter;
  private final DoubleSupplier rpmShooterTop, rpmShooterBottom;
  /** Creates a new PrimeShooter. */
  public PrimeShooter(Shooter sysShooter, DoubleSupplier rpmShooterTop, DoubleSupplier rpmShooterBottom) {
    this.sysShooter = sysShooter;
    this.rpmShooterTop = rpmShooterTop;
    this.rpmShooterBottom = rpmShooterBottom;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysShooter.runShooterVelocity(rpmShooterTop.getAsDouble(), rpmShooterBottom.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    sysShooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
