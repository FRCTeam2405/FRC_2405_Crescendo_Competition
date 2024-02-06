// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooting;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem {
  //TODO! Give these relevant names based on their position
  CANSparkFlex shooterOne;
  CANSparkFlex shooterTwo;

  /** Creates a new Launcher. */
  public Shooter() {
    //TODO! ports
    shooterOne = new CANSparkFlex(0, MotorType.kBrushless);
    shooterTwo = new CANSparkFlex(0, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
