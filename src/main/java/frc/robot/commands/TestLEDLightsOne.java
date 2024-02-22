// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDLights;

public class TestLEDLightsOne extends Command {
  /** Creates a new TestLEDLights. */
  
  final LEDLights sysLEDLights;
  final double LEDSetting;
  
  public TestLEDLightsOne(LEDLights insysLEDLights, double inLEDSetting) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysLEDLights = insysLEDLights;
    LEDSetting = inLEDSetting;
  
    // Use addRequirements() here to declare subsystem dependencies.  
    addRequirements(insysLEDLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sysLEDLights.LEDLightsOne.set(SmartDashboard.getNumber("LEDSetOne", Constants.LEDs.LED_COLORS.LED_SETTING_DEFAULT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysLEDLights.LEDLightsOne.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sysLEDLights.LEDLightsOne.get() == SmartDashboard.getNumber("LEDSetOne", Constants.LEDs.LED_COLORS.LED_SETTING_DEFAULT))
      return true; 
    else
      return false;
  }
}
