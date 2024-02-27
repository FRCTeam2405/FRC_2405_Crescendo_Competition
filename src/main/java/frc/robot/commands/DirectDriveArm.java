// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class DirectDriveArm extends Command {

  private final Arm sysArm;
  private final DoubleSupplier positionArm;

  /** Creates a new DirectDriveArm. */
  public DirectDriveArm(Arm sysArm, DoubleSupplier positionArm) {

    this.sysArm = sysArm;
    this.positionArm = positionArm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = sysArm.getArmPosition();
    sysArm.MoveArmToPosition(currentPosition + (positionArm.getAsDouble() * Constants.Arm.DIRECT_DRIVE_MOD));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
