// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooting.Intake;

public class IntakeNote extends Command {

  private final Intake sysIntake;
  private final DoubleSupplier speedIntakeRight;
  

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake sysIntake) {

    this.sysIntake = sysIntake;
    this.speedIntakeRight = () -> Constants.Intake.Motors.RIGHT_INTAKE_SPEED_MAX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysIntake.runIntake(speedIntakeRight.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    sysIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
