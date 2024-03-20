// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooting;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem {
  private CANSparkFlex motorTopShooter, motorBottomShooter;
  private SparkPIDController pidTopShooter, pidBottomShooter;
  private RelativeEncoder encoderTopShooter, encoderBottomShooter;

  /** Creates a new Launcher. */
  public Shooter() {
  
    motorTopShooter = new CANSparkFlex(Constants.Shooter.Motors.TOP_SHOOTER_PORTID, MotorType.kBrushless);
    motorBottomShooter = new CANSparkFlex(Constants.Shooter.Motors.BOTTOM_SHOOTER_PORTID, MotorType.kBrushless);

    pidTopShooter = motorTopShooter.getPIDController();
    pidBottomShooter = motorBottomShooter.getPIDController();

    encoderTopShooter = motorTopShooter.getEncoder();
    encoderBottomShooter = motorBottomShooter.getEncoder();

    pidTopShooter.setP(Constants.Shooter.Encoder.GAIN_P);
    pidTopShooter.setI(Constants.Shooter.Encoder.GAIN_I);
    pidTopShooter.setD(Constants.Shooter.Encoder.GAIN_D);
    pidTopShooter.setIZone(Constants.Shooter.Encoder.ZONE_I);
    pidTopShooter.setFF(Constants.Shooter.Encoder.FEED_FORWARD);
    pidTopShooter.setOutputRange(Constants.Shooter.Encoder.OUTPUT_RANGE_MIN, Constants.Shooter.Encoder.OUTPUT_RANGE_MAX);

    pidBottomShooter.setP(Constants.Shooter.Encoder.GAIN_P);
    pidBottomShooter.setI(Constants.Shooter.Encoder.GAIN_I);
    pidBottomShooter.setD(Constants.Shooter.Encoder.GAIN_D);
    pidBottomShooter.setIZone(Constants.Shooter.Encoder.ZONE_I);
    pidBottomShooter.setFF(Constants.Shooter.Encoder.FEED_FORWARD);
    pidBottomShooter.setOutputRange(Constants.Shooter.Encoder.OUTPUT_RANGE_MIN, Constants.Shooter.Encoder.OUTPUT_RANGE_MAX);

    motorTopShooter.setInverted(Constants.Shooter.Motors.TOP_SHOOTER_INVERTED);
    motorBottomShooter.setInverted(Constants.Shooter.Motors.BOTTOM_SHOOTER_INVERTED);
  }

  public void runShooterPercentOutput(double percentOutputTop, double percentOutputBottom) {
    motorTopShooter.set(percentOutputTop);
    motorBottomShooter.set(percentOutputBottom);

  }

  public void runShooterVelocity(double rpmTop, double rpmBottom) {
    pidTopShooter.setReference(rpmTop, ControlType.kVelocity);
    pidBottomShooter.setReference(rpmBottom, ControlType.kVelocity);

  }

  public void runShooterVelocity() {
    pidTopShooter.setReference(Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_JOY, ControlType.kVelocity);
    pidBottomShooter.setReference(Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_JOY, ControlType.kVelocity);

  }

  public void stopShooter() {
    motorTopShooter.set(0);
    motorBottomShooter.set(0);
  }

  

  public double getShooterVelocity(int motorId) {
    switch(motorId){
      case Constants.Shooter.Motors.TOP_SHOOTER_PORTID:
        return encoderTopShooter.getVelocity();
      case Constants.Shooter.Motors.BOTTOM_SHOOTER_PORTID:
        return encoderBottomShooter.getVelocity();
      default:
        return 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
