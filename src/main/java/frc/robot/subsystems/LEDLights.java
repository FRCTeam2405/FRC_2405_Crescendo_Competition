// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDLights extends SubsystemBase {

  private PWMSparkMax LEDLightsOne;
  private PWMSparkMax LEDLightsTwo;

  private boolean isEndgame = false;

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
    SmartDashboard.putNumber(Constants.Dashboard.Utility.Widgets.ROBOT_EMOTION_SETTING_NAME, 0);
    
    // we force an endgame color when there are 30 seconds left in teleop
    if(!isEndgame && DriverStation.isTeleop() && Timer.getMatchTime() <= 30.0) {
      isEndgame = true;

      LEDLightsOne.set(Constants.LEDs.LED_COLORS.SOLID_RED);
      LEDLightsTwo.set(Constants.LEDs.LED_COLORS.SOLID_RED);
    }

    // This method will be called once per scheduler run
  }

  public void setColorOne(double Color) {
    if(isEndgame) {
      Color = Constants.LEDs.LED_COLORS.SOLID_RED;
    }
    LEDLightsOne.set(Color);
  }
  public void setColorTwo(double Color) {
    if(isEndgame) {
      Color = Constants.LEDs.LED_COLORS.SOLID_RED;
    }
    LEDLightsTwo.set(Color);
  }

  public void setColorBoth(double color) {
    if(isEndgame) {
      color = Constants.LEDs.LED_COLORS.SOLID_RED;
    }
    LEDLightsOne.set(color);
    LEDLightsTwo.set(color);
  }

  public void setColorBoth(double colorOne, double colorTwo) {
    if(isEndgame) {
      colorOne = Constants.LEDs.LED_COLORS.SOLID_RED;
      colorTwo = Constants.LEDs.LED_COLORS.SOLID_RED;
    }
    LEDLightsOne.set(colorOne);
    LEDLightsTwo.set(colorTwo);
  }

  public void setRobotEmotion() {
    double emotionSetting = SmartDashboard.getNumber(Constants.Dashboard.Utility.Widgets.ROBOT_EMOTION_SETTING_NAME, 0);

    if (emotionSetting == Constants.Controllers.Guitar.ROBOT_EMOTION_SETTING.SADNESS) {
      LEDLightsOne.set(Constants.LEDs.ROBOT_EMOTIONS.SADNESS_COLOR_ONE);
      LEDLightsTwo.set(Constants.LEDs.ROBOT_EMOTIONS.SADNESS_COLOR_TWO);
    }
    else if (emotionSetting == Constants.Controllers.Guitar.ROBOT_EMOTION_SETTING.FEAR) {
      LEDLightsOne.set(Constants.LEDs.ROBOT_EMOTIONS.FEAR_COLOR_ONE);
      LEDLightsTwo.set(Constants.LEDs.ROBOT_EMOTIONS.FEAR_COLOR_TWO);
    }
    else if (emotionSetting == Constants.Controllers.Guitar.ROBOT_EMOTION_SETTING.JOY) {
      LEDLightsOne.set(Constants.LEDs.ROBOT_EMOTIONS.JOY_COLOR_ONE);
      LEDLightsTwo.set(Constants.LEDs.ROBOT_EMOTIONS.JOY_COLOR_TWO);
    }
    else if (emotionSetting == Constants.Controllers.Guitar.ROBOT_EMOTION_SETTING.DISGUST) {
      LEDLightsOne.set(Constants.LEDs.ROBOT_EMOTIONS.DISGUST_COLOR_ONE);
      LEDLightsTwo.set(Constants.LEDs.ROBOT_EMOTIONS.DISGUST_COLOR_TWO);
    }
    else if (emotionSetting == Constants.Controllers.Guitar.ROBOT_EMOTION_SETTING.ANGER) {
      LEDLightsOne.set(Constants.LEDs.ROBOT_EMOTIONS.ANGER_COLOR_ONE);
      LEDLightsTwo.set(Constants.LEDs.ROBOT_EMOTIONS.ANGER_COLOR_TWO);
    }
    else {
      LEDLightsOne.set(Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT);
      LEDLightsTwo.set(Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
    }
  }

  public double GetColorOne() {
    return LEDLightsOne.get();
  }
  public double GetColorTwo() {
    return LEDLightsTwo.get();
  }
}
