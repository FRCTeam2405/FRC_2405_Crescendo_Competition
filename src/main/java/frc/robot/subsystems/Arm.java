// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private PositionVoltage motorPositionVoltage;
  private NeutralOut motorBreak;
  private TalonFXConfiguration motorArmConfig;

  /** Creates a new Arm. */
  public Arm() {

    motorArm = new TalonFX(Constants.Arm.Motors.ARM_PORTID);
    motorArm.setInverted(Constants.Arm.Motors.ARM_INVERTED);
    motorArm.setNeutralMode(NeutralModeValue.Brake);

    motorPositionVoltage = new PositionVoltage(
                            0, 
                            0, 
                            false, 
                            0, 
                            Constants.Arm.Encoder.ProfileZero.PROFILE_ID, 
                            false, 
                            false, 
                            false);

    motorBreak = new NeutralOut();
    motorArmConfig = new TalonFXConfiguration();
    motorArmConfig.Slot0.kP = Constants.Arm.Encoder.ProfileZero.GAIN_P;
    motorArmConfig.Slot0.kD = Constants.Arm.Encoder.ProfileZero.GAIN_D;
    motorArmConfig.Voltage.PeakForwardVoltage = Constants.Arm.Encoder.ProfileZero.PEAK_FORWARD_VOLTAGE;
    motorArmConfig.Voltage.PeakReverseVoltage = Constants.Arm.Encoder.ProfileZero.PEAK_REVERSE_VOLTAGE;
    motorArm.getConfigurator().apply(motorArmConfig);
    motorArm.setPosition(0);
  }

  public void moveArmToPosition(double newSetPoint) {

    motorArm.setControl(motorPositionVoltage.withPosition(-newSetPoint));
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
