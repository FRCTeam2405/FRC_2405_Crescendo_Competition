// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDLights;

public class TestLEDLightsTwo extends Command {

  private final LEDLights sysLighting;
  private final double LEDSetting;
  
  /** Test command for the second LED strip on the robot. */
  public TestLEDLightsTwo(LEDLights insysLEDLights, double inLEDSetting) {
    sysLighting = insysLEDLights;
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
    // Set the LED to the setting on the dashboard.
    sysLighting.setColorTwo(SmartDashboard.getNumber("LEDSetTwo", Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysLighting.setColorTwo(Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
