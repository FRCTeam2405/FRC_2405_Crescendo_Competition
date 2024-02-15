// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooting.Feeder;
import frc.robot.subsystems.shooting.Shooter;

public class FireWhenReadyVelocity extends Command {

  private final Shooter sysShooter;
  private final Feeder sysFeeder;
  private final DoubleSupplier rpmShooterTop, rpmShooterBottom, percentOutputFeederTop, percentOutputFeederBottom;
  /** Creates a new FireWhenReadyVelocity. */
  public FireWhenReadyVelocity(Shooter sysShooter, Feeder sysFeeder,
                      DoubleSupplier rpmShooterTop, DoubleSupplier rpmShooterBottom, 
                      DoubleSupplier percentOutputFeederTop, DoubleSupplier percentOutputFeederBottom)  {
    this.sysShooter = sysShooter;
    this.sysFeeder = sysFeeder;
    this.rpmShooterTop = rpmShooterTop;
    this.rpmShooterBottom = rpmShooterBottom;
    this.percentOutputFeederTop = percentOutputFeederTop;
    this.percentOutputFeederBottom = percentOutputFeederBottom;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysShooter, sysFeeder);
  }

  public FireWhenReadyVelocity(Shooter sysShooter, Feeder sysFeeder, 
                      DoubleSupplier rpmShooter, 
                      DoubleSupplier percentOutputFeeder)  {
    this.sysShooter = sysShooter;
    this.sysFeeder = sysFeeder;
    this.rpmShooterTop = rpmShooter;
    this.rpmShooterBottom = rpmShooter;
    this.percentOutputFeederTop = percentOutputFeeder;
    this.percentOutputFeederBottom = percentOutputFeeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysShooter, sysFeeder);
  }

  public FireWhenReadyVelocity(Shooter sysShooter, Feeder sysFeeder)  {
    this.sysShooter = sysShooter;
    this.sysFeeder = sysFeeder;
    this.rpmShooterTop = () -> Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_DEFAULT;
    this.rpmShooterBottom = () -> Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_DEFAULT;
    this.percentOutputFeederTop = () -> Constants.Feeder.Motors.TOP_FEEDER_SPEED_MAX;
    this.percentOutputFeederBottom = () -> Constants.Feeder.Motors.BOTTOM_FEEDER_SPEED_MAX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysShooter, sysFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysShooter.runShooterVelocity(rpmShooterTop.getAsDouble(), rpmShooterBottom.getAsDouble());
    
    if (sysShooter.getShooterVelocity(Constants.Shooter.Motors.TOP_SHOOTER_PORTID) 
                >= rpmShooterTop.getAsDouble() && 
        sysShooter.getShooterVelocity(Constants.Shooter.Motors.BOTTOM_SHOOTER_PORTID)
                >= rpmShooterBottom.getAsDouble()) {
      
      sysFeeder.runFeeder(percentOutputFeederTop.getAsDouble(), percentOutputFeederBottom.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    sysShooter.stopShooter();
    sysFeeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
