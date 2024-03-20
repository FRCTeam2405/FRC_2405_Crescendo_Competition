// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotEmotion;

public class SetRobotEmotion extends Command {

  private final RobotEmotion sysRobotEmotion;
  private final DoubleSupplier emotionSetting;

  /** Creates a new SetRobotEmotion. */
  public SetRobotEmotion(RobotEmotion sysRobotEmotion, DoubleSupplier emotionSetting) {

    this.sysRobotEmotion = sysRobotEmotion;
    this.emotionSetting = emotionSetting;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysRobotEmotion);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysRobotEmotion.setRobotEmotion(emotionSetting.getAsDouble());
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
