// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooting.Intake;
import frc.robot.subsystems.shooting.Feeder;

public class IntakeNote extends Command {

  private final Intake sysIntake;
  private final Feeder sysFeeder;
  private final DoubleSupplier speedIntakeRight, speedFeederTop, speedFeederBottom;
  

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake sysIntake, Feeder sysFeeder) {

    this.sysIntake = sysIntake;
    this.sysFeeder = sysFeeder;
    this.speedIntakeRight = () -> Constants.Intake.Motors.RIGHT_INTAKE_SPEED_MAX;
    this.speedFeederTop = () -> Constants.Feeder.Motors.TOP_FEEDER_SPEED_MAX;
    this.speedFeederBottom = () -> Constants.Feeder.Motors.BOTTOM_FEEDER_SPEED_MAX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake, sysFeeder);
  }

  public IntakeNote(Intake sysIntake, Feeder sysFeeder, DoubleSupplier speedIntakeRight, DoubleSupplier speedFeederTop, DoubleSupplier speedFeederBottom) {

    this.sysIntake = sysIntake;
    this.sysFeeder = sysFeeder;
    this.speedIntakeRight = speedIntakeRight;
    this.speedFeederTop = speedFeederTop;
    this.speedFeederBottom = speedFeederBottom;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake, sysFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysIntake.runIntake(speedIntakeRight.getAsDouble());
    sysFeeder.runFeeder(speedFeederTop.getAsDouble(), speedFeederBottom.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    sysIntake.stopIntake();
    sysFeeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
