// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooting.Shooter;
import frc.robot.Constants;
import frc.robot.RobotEmotionState;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.RobotEmotion;

public class PrimeShooter extends Command {

  private final Shooter sysShooter;
  private final RobotEmotion sysRobotEmotion;
  private final LEDLights sysLighting;
  private final Dashboard sysDashboard;
  private final DoubleSupplier rpmShooterTop, rpmShooterBottom;

  /** Creates a new PrimeShooter. */
  public PrimeShooter(Shooter sysShooter, RobotEmotion sysRobotEmotion, LEDLights sysLighting, Dashboard sysDashboard, DoubleSupplier rpmShooterTop, DoubleSupplier rpmShooterBottom) {
    this.sysShooter = sysShooter;
    this.sysRobotEmotion = sysRobotEmotion;
    this.sysLighting = sysLighting;
    this.sysDashboard = sysDashboard;
    this.rpmShooterTop = rpmShooterTop;
    this.rpmShooterBottom = rpmShooterBottom;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysShooter, sysRobotEmotion, sysLighting, sysDashboard);
  }

  public PrimeShooter(Shooter sysShooter, RobotEmotion sysRobotEmotion, LEDLights sysLighting, Dashboard sysDashboard, boolean useRobotEmotion) {
    this.sysShooter = sysShooter;
    this.sysRobotEmotion = sysRobotEmotion;
    this.sysLighting = sysLighting;
    this.sysDashboard = sysDashboard;

    if (useRobotEmotion) {
      this.rpmShooterTop = () -> sysRobotEmotion.getEmotionTopShooterVelocity();
      this.rpmShooterBottom = () -> sysRobotEmotion.getEmotionBottomShooterVelocity();
    }
    else {
    this.rpmShooterTop = () -> sysDashboard.getTopShooterVelocityDashboard();
    this.rpmShooterBottom = () -> sysDashboard.getBottomShooterVelocityDashboard();
    }
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sysShooter, sysRobotEmotion, sysLighting, sysDashboard);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sysShooter.runShooterVelocity(rpmShooterTop.getAsDouble(), rpmShooterBottom.getAsDouble());
    sysLighting.setColorBoth(Constants.LEDs.LED_COLORS.SHOOTER_COLOR_ONE, Constants.LEDs.LED_COLORS.SHOOTER_COLOR_TWO);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    sysShooter.stopShooter();
    sysLighting.setRobotEmotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
