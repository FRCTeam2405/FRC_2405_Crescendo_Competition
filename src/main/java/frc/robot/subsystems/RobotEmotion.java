// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
      case 0, 1:
        robotEmotion = robotEmotion.Sadness;
        break;
      
      case 2, 3:
        robotEmotion = robotEmotion.Fear;
        break;

      case 4, 5:
        robotEmotion = robotEmotion.Joy;
        break;

      case 6, 7:
        robotEmotion = robotEmotion.Disgust;
        break;

      case 8, 9, 10:
        robotEmotion = robotEmotion.Anger;
        break;

    
      default:
        robotEmotion = robotEmotion.Sadness;
        break;
    }
  }

  public RobotEmotionState getRobotEmotion() {
    return robotEmotion;
  }

  public Command setEmotionCommand(DoubleSupplier robotEmotion) {
    return run(() -> {
      this.setRobotEmotion(robotEmotion.getAsDouble());
    });
  }

public double getEmotionTopShooterVelocity() {
    return robotEmotion.getEmotionTopShooterVelocity();
}

public double getEmotionBottomShooterVelocity() {
    return robotEmotion.getEmotionBottomShooterVelocity();
}

public double getEmotionLightingColorOne() {
    return robotEmotion.getEmotionLightingColorOne();
}

public double getEmotionLightingColorTwo() {
    return robotEmotion.getEmotionLightingColorTwo();
}

public double getEmotionArmPosition() {
    return robotEmotion.getEmotionArmPosition();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("RobotEmotionSetPoint", robotEmotion.getEmotionSetPoint());
    SmartDashboard.putString("RobotEmotionName", robotEmotion.name());
  }
}
