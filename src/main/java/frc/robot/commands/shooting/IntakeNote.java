// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooting.Intake;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.shooting.Feeder;

public class IntakeNote extends Command {

  private final Intake sysIntake;
  private final Feeder sysFeeder;
  private final LEDLights sysLighting;
  private final Dashboard sysDashboard;
  private final DoubleSupplier speedIntakeRight, speedFeederTop, speedFeederBottom;
  

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake sysIntake, Feeder sysFeeder, LEDLights sysLighting, Dashboard sysDashboard) {

    this.sysIntake = sysIntake;
    this.sysFeeder = sysFeeder;
    this.sysLighting = sysLighting;
    this.sysDashboard = sysDashboard;
    this.speedIntakeRight = () -> sysDashboard.getRightIntakeSpeedDashboard();
    this.speedFeederTop = () -> sysDashboard.getTopFeederIntakingSpeedDashboard();
    this.speedFeederBottom = () -> sysDashboard.getBottomFeederIntakingSpeedDashboard();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake, sysFeeder, sysLighting, sysDashboard);
  }

  public IntakeNote(Intake sysIntake, Feeder sysFeeder, LEDLights sysLighting, Dashboard sysDashboard, DoubleSupplier speedIntakeRight, DoubleSupplier speedFeederTop, DoubleSupplier speedFeederBottom) {

    this.sysIntake = sysIntake;
    this.sysFeeder = sysFeeder;
    this.sysLighting = sysLighting;
    this.sysDashboard = sysDashboard;
    this.speedIntakeRight = speedIntakeRight;
    this.speedFeederTop = speedFeederTop;
    this.speedFeederBottom = speedFeederBottom;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake, sysFeeder, sysLighting, sysDashboard);
  }

  public IntakeNote(Intake sysIntake, Feeder sysFeeder, LEDLights sysLighting, Dashboard sysDashboard, DoubleSupplier speedIntakeRight, DoubleSupplier speedFeeder) {

    this.sysIntake = sysIntake;
    this.sysFeeder = sysFeeder;
    this.sysLighting = sysLighting;
    this.sysDashboard = sysDashboard;
    this.speedIntakeRight = speedIntakeRight;
    this.speedFeederTop = speedFeeder;
    this.speedFeederBottom = speedFeeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysIntake, sysFeeder, sysLighting, sysDashboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!sysFeeder.getNoteLimit()) {
      sysIntake.runIntake(speedIntakeRight.getAsDouble());
      sysFeeder.runFeeder(speedFeederTop.getAsDouble(), speedFeederBottom.getAsDouble());
      if (speedIntakeRight.getAsDouble() < 0) {
        sysLighting.setColorBoth(Constants.LEDs.LED_COLORS.INTAKE_REVERSE_COLOR_ONE, Constants.LEDs.LED_COLORS.INTAKE_REVERSE_COLOR_TWO);  
      }
      else {
        sysLighting.setColorBoth(Constants.LEDs.LED_COLORS.INTAKE_COLOR_ONE, Constants.LEDs.LED_COLORS.INTAKE_COLOR_TWO);
      }
      
    }
    else {
      sysIntake.stopIntake();
      sysFeeder.stopFeeder();
      sysLighting.setColorBoth(Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT, Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    sysIntake.stopIntake();
    sysFeeder.stopFeeder();
    sysLighting.setColorBoth(Constants.LEDs.LED_COLORS.TELEOP_COLOR_ONE_DEFAULT, Constants.LEDs.LED_COLORS.TELEOP_COLOR_TWO_DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled() && sysFeeder.getNoteLimit() == true) {
      return true;
    } else {
     return false;
    }
  }
}
