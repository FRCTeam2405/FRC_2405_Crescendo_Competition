// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooting;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
  //TODO! Give these relevant names based on their position
  CANSparkMax intakeOne;
  CANSparkMax intakeTwo;

  /** Creates a new Intake. */
  public Intake() {
    //TODO! ports
    intakeOne = new CANSparkMax(0, MotorType.kBrushless);
    intakeTwo = new CANSparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
