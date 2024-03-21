// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public enum RobotEmotionState {
    Sadness(
            1,
            Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_SADNESS,
            Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_SADNESS,
            Constants.LEDs.ROBOT_EMOTIONS.SADNESS_COLOR_ONE,
            Constants.LEDs.ROBOT_EMOTIONS.SADNESS_COLOR_TWO),
    Fear(
            3,
            Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_FEAR,
            Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_FEAR,
            Constants.LEDs.ROBOT_EMOTIONS.FEAR_COLOR_ONE,
            Constants.LEDs.ROBOT_EMOTIONS.FEAR_COLOR_TWO),
    Joy(
            5,
            Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_JOY,
            Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_JOY,
            Constants.LEDs.ROBOT_EMOTIONS.JOY_COLOR_ONE,
            Constants.LEDs.ROBOT_EMOTIONS.JOY_COLOR_TWO),
    Disgust(   
            7,
            Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_DISGUST,
            Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_DISGUST,
            Constants.LEDs.ROBOT_EMOTIONS.DISGUST_COLOR_ONE,
            Constants.LEDs.ROBOT_EMOTIONS.DISGUST_COLOR_TWO),
    Anger(
            9,
            Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_ANGER,
            Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_ANGER,
            Constants.LEDs.ROBOT_EMOTIONS.ANGER_COLOR_ONE,
            Constants.LEDs.ROBOT_EMOTIONS.ANGER_COLOR_TWO);


    private double setPoint, motorTopShooterVelocity, motorBottomShooterVelocity, lightingColorOne, lightingColorTwo;
    
    RobotEmotionState(double setPoint, double motorTopShooterVelocity, double motorBottomShooterVelocity, double lightingColorOne, double lightingColorTwo) {
        this.setPoint = setPoint;
        this.motorTopShooterVelocity = motorTopShooterVelocity;
        this.motorBottomShooterVelocity = motorBottomShooterVelocity;
        this.lightingColorOne = lightingColorOne;
        this.lightingColorTwo = lightingColorTwo;
    }

    


    public double getEmotionSetPoint() {
        return setPoint;
    }

    public double getEmotionTopShooterVelocity() {
        return motorTopShooterVelocity;
    }

    public double getEmotionBottomShooterVelocity() {
        return motorBottomShooterVelocity;
    }

    public double getEmotionLightingColorOne() {
        return lightingColorOne;
    }

    public double getEmotionLightingColorTwo() {
        return lightingColorTwo;
    }
}

