// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotEmotionState;

public class RobotEmotion extends SubsystemBase {

  private RobotEmotionState robotEmotion;

  /** Creates a new RobotEmotion. */
  public RobotEmotion() {
    robotEmotion = robotEmotion.Sadness;
  }

  public void setRobotEmotion(double setPoint) {

    int convertSetPointCheck = (int)Math.round(setPoint * 10.0);

    switch (convertSetPointCheck) {
      case 1:
        robotEmotion = robotEmotion.Sadness;
        break;
      
      case 3:
        robotEmotion = robotEmotion.Fear;
        break;

      case 5:
        robotEmotion = robotEmotion.Joy;
        break;

      case 7:
        robotEmotion = robotEmotion.Disgust;
        break;

      case 9:
        robotEmotion = robotEmotion.Anger;
        break;

    
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("RobotEmotionSetPoint", robotEmotion.getEmotionSetPoint());
    SmartDashboard.putString("RobotEmotionName", robotEmotion.name());
  }
}
