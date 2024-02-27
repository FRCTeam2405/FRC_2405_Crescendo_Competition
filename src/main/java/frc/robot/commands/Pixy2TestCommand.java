// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pixy2_library.Pixy2CCC.Block;
import frc.robot.subsystems.PixyCam;

public class Pixy2TestCommand extends Command {

  private PixyCam pixyCam;

  /** Creates a new Pixy2TestCommand. */
  public Pixy2TestCommand(PixyCam pixyCam) {
    this.pixyCam = pixyCam;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pixyCam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ArrayList<Block> blocks = pixyCam.getBlocks();
    if(blocks.isEmpty()) {
      return;
    }
    SmartDashboard.putNumber("firstBlockX", blocks.get(0).getX());
    SmartDashboard.putNumber("firstBlockY", blocks.get(0).getY());
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
