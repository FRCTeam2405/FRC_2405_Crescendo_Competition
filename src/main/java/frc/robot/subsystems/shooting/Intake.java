// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooting;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
  private CANSparkMax motorIntakeRight;

  /** Creates a new Intake. */
  public Intake() {
    motorIntakeRight = new CANSparkMax(Constants.Intake.Motors.RIGHT_INTAKE_PORTID, MotorType.kBrushless);
    motorIntakeRight.setInverted(Constants.Intake.Motors.RIGHT_INTAKE_INVERTED);

  }

  public void runIntake(double speedRight) {
    motorIntakeRight.set(speedRight);
  }

  public void runIntake() {
    motorIntakeRight.set(Constants.Intake.Motors.RIGHT_INTAKE_SPEED_MAX);
  }

  public void stopIntake() {
    motorIntakeRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
