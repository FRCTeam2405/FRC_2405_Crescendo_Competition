// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooting;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class NoteHolder implements Subsystem {
  //TODO! Give this a relevant name
  CANSparkMax feedForwardMotor;

  /** Creates a new NoteHolder. */
  public NoteHolder() {
    //TODO! port
    feedForwardMotor = new CANSparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
