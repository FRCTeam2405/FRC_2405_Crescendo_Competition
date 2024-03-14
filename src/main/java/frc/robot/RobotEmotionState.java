// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum RobotEmotionState {
    Sadness(0.1, 1),
    Fear(0.3, 3),
    Joy(0.5, 5),
    Disgust(0.7, 7),
    Anger(0.9, 9);

    RobotEmotionState(double setPoint, int convertSetPoint) {
        this.setPoint = setPoint;
        this.convertSetPoint = convertSetPoint;
    }

    private double setPoint;
    private int convertSetPoint;


    public double getEmotionSetPoint() {
        return setPoint;
    }

    public int getEmotionConvertSetPoint() {
        return convertSetPoint;
    }
}

