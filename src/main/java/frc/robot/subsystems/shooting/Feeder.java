// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooting;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Feeder implements Subsystem {
  private CANSparkFlex motorTopFeeder, motorBottomFeeder;
  private DigitalInput noteLimit;

  /** Creates a new NoteHolder. */
  public Feeder() {
    motorTopFeeder = new CANSparkFlex(Constants.Feeder.Motors.TOP_FEEDER_PORTID, MotorType.kBrushless);
    motorBottomFeeder = new CANSparkFlex(Constants.Feeder.Motors.BOTTOM_FEEDER_PORTID, MotorType.kBrushless);

    motorTopFeeder.setInverted(Constants.Feeder.Motors.TOP_FEEDER_INVERTED);
    motorBottomFeeder.setInverted(Constants.Feeder.Motors.BOTTOM_FEEDER_INVERTED);
    noteLimit = new DigitalInput(Constants.Feeder.Sensors.NOTE_LIMIT_PORTID);
  }

  public void runFeeder(double speedTop, double speedBottom) {
    motorTopFeeder.set(speedTop);
    motorBottomFeeder.set(speedBottom);
  }

  public void stopFeeder() {
    motorTopFeeder.set(0);
    motorBottomFeeder.set(0);
  }

  public boolean getNoteLimit() {

    return noteLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
