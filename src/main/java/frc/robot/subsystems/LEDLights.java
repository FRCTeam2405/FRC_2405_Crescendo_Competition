// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDLights extends SubsystemBase {

  private PWMSparkMax LEDLightsOne;
  private PWMSparkMax LEDLightsTwo;

  /** Creates a new LEDLights. */
  public LEDLights() {
    LEDLightsOne = new PWMSparkMax(Constants.LEDs.LED_PORT_ONE);
    LEDLightsTwo = new PWMSparkMax(Constants.LEDs.LED_PORT_TWO);
    SmartDashboard.putNumber("LEDSetOne", Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT);
    SmartDashboard.putNumber("LEDSetTwo", Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
  }

  public LEDLights(double colorOne, double colorTwo) {
    LEDLightsOne = new PWMSparkMax(Constants.LEDs.LED_PORT_ONE);
    LEDLightsTwo = new PWMSparkMax(Constants.LEDs.LED_PORT_TWO);
    LEDLightsOne.set(colorOne);
    LEDLightsTwo.set(colorTwo);
    SmartDashboard.putNumber("LEDSetOne", Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT);
    SmartDashboard.putNumber("LEDSetTwo", Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColorOne(double Color) {
    LEDLightsOne.set(Color);
  }
  public void setColorTwo(double Color) {
    LEDLightsTwo.set(Color);
  }

  public void setColorBoth(double color) {
    LEDLightsOne.set(color);
    LEDLightsTwo.set(color);
  }

  public void setColorBoth(double colorOne, double colorTwo) {
    LEDLightsOne.set(colorOne);
    LEDLightsTwo.set(colorTwo);
  }

  public double GetColorOne() {
    return LEDLightsOne.get();
  }
  public double GetColorTwo() {
    return LEDLightsTwo.get();
  }
}
