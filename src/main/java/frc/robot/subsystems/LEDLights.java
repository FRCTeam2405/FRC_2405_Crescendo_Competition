// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDLights extends SubsystemBase {

  public PWMSparkMax LEDLightsOne = new PWMSparkMax(Constants.LEDs.LED_PORT_ONE);
  public PWMSparkMax LEDLightsTwo = new PWMSparkMax(Constants.LEDs.LED_PORT_TWO);

  /** Creates a new LEDLights. */
  public LEDLights() {
    SmartDashboard.putNumber("LEDSetOne", Constants.LEDs.LED_COLORS.LED_SETTING_DEFAULT);
    SmartDashboard.putNumber("LEDSetTwo", Constants.LEDs.LED_COLORS.LED_SETTING_DEFAULT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetColorOne(double Color) {
    LEDLightsOne.set(Color);
  }
  public void SetColorTwo(double Color) {
    LEDLightsTwo.set(Color);
  }

  public double GetColorOne() {
    return LEDLightsOne.get();
  }
  public double GetColorTwo() {
    return LEDLightsTwo.get();
  }
}
