// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDLights;

public class TestLEDLightsOne extends Command {
  
  private final LEDLights sysLighting;
  private final double LEDSetting;
  
  /** Test command for the first LED strip on the robot. */
  public TestLEDLightsOne(LEDLights sysLighting, double LEDSetting) {
    
    this.sysLighting = sysLighting;
    this.LEDSetting = LEDSetting;
  
    // Use addRequirements() here to declare subsystem dependencies.  
    addRequirements(sysLighting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the LED to the setting on the dashboard.
    sysLighting.setColorOne(SmartDashboard.getNumber("LEDSetOne", Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysLighting.setColorOne(Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
