// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private TalonFX motorArm;
  private Slot0Configs motorArmConfig;
  private PositionVoltage motorPositionVoltage;
  private NeutralOut motorBreak;

  /** Creates a new Arm. */
  public Arm() {

    motorArm = new TalonFX(Constants.Arm.Motors.ARM_PORTID);
    motorArm.setInverted(Constants.Arm.Motors.ARM_INVERTED);
    motorArm.setNeutralMode(NeutralModeValue.Brake);

    

    motorBreak = new NeutralOut();
    motorArmConfig = new Slot0Configs();
    motorArmConfig.kP = Constants.Arm.Encoder.ProfileZero.GAIN_P;
    motorArmConfig.kD = Constants.Arm.Encoder.ProfileZero.GAIN_D;
    motorArm.getConfigurator().apply(motorArmConfig);
    motorPositionVoltage = new PositionVoltage(0).withSlot(Constants.Arm.Encoder.ProfileZero.PROFILE_ID);
  }

  public void moveArmToPosition(double newSetPoint) {

    motorArm.setControl(motorPositionVoltage.withPosition(newSetPoint));
  }

  public double getArmPosition() {
    return motorArm.getPosition().getValue();
  }

  public void setMotorControlBreak() {
    motorArm.setControl(motorBreak);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
