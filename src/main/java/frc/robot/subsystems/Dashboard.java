// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooting.Feeder;
import frc.robot.subsystems.shooting.Intake;
import frc.robot.subsystems.shooting.Shooter;

public class Dashboard extends SubsystemBase {

  private final Shooter sysShooter;
  private final Feeder sysFeeder;
  private final Intake sysIntake;
  private final Arm sysArm;
  private ShuffleboardTab dashboardTabMain, dashboardTabUtility;
  private GenericEntry  dashboardEntryTopShooterVelocity, dashboardEntryBottomShooterVelocity, 
                        dashboardEntryTopShooterVelocitySetting, dashboardEntryBottomShooterVelocitySetting,
                        dashboardEntryTopFeederShootingSpeedSetting, dashboardEntryBottomFeederShootingSpeedSetting,
                        dashboardEntryTopFeederIntakingSpeedSetting, dashboardEntryBottomFeederIntakingSpeedSetting,
                        dashboardEntryRightIntakeSpeed, dashboardEntryArmVelocitySetting,
                        dashboardEntryArmPosition, dashboardEntryArmPositionSetting,
                        dashboardEntryFeederNoteLimit, dashboardEntryPhantom;

  /** Creates a new Dashboard. */
  public Dashboard(Shooter sysShooter, Feeder sysFeeder, Intake sysIntake, Arm sysArm) {

    this.sysShooter = sysShooter;
    this.sysFeeder = sysFeeder;
    this.sysIntake = sysIntake;
    this.sysArm = sysArm;

    dashboardTabMain = Shuffleboard.getTab(Constants.Dashboard.Main.TAB_NAME);
    dashboardTabUtility = Shuffleboard.getTab(Constants.Dashboard.Utility.TAB_NAME);
    setDashboardUtility();
  }

  private void setDashboardMain() {

  }

  private void setDashboardUtility() {
    // Shooter widgets
    dashboardEntryTopShooterVelocitySetting = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.TOP_SHOOTER_VELOCITY_SETTING_NAME, 
              Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_DEFAULT)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", Constants.Shooter.Motors.TOP_SHOOTER_VELOCITY_MAX))
              .withPosition(0,0)
              .withSize(2, 1)
              .getEntry();
    dashboardEntryBottomShooterVelocitySetting = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.BOTTOM_SHOOTER_VELOCITY_SETTING_NAME, 
              Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_DEFAULT)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withPosition(0,1)
              .withProperties(Map.of("min", 0, "max", Constants.Shooter.Motors.BOTTOM_SHOOTER_VELOCITY_MAX))
              .withSize(2, 1)
              .getEntry();
    dashboardEntryTopShooterVelocity = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.TOP_SHOOTER_VELOCITY_NAME, 
              0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(2,0)
              .withSize(1, 1)
              .getEntry();
    dashboardEntryBottomShooterVelocity = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.BOTTOM_SHOOTER_VELOCITY_NAME, 
              0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(2,1)
              .withSize(1, 1)
              .getEntry();
    // Feeder widgets
    dashboardEntryTopFeederShootingSpeedSetting = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.TOP_FEEDER_SHOOTING_OUTPUT_SETTING_NAME, 
                Constants.Feeder.Motors.TOP_FEEDER_SHOOTING_SPEED)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withPosition(3,0)
                .withSize(2, 1)
                .getEntry();
    dashboardEntryBottomFeederShootingSpeedSetting = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.BOTTOM_FEEDER_SHOOTING_OUTPUT_SETTING_NAME,
                Constants.Feeder.Motors.BOTTOM_FEEDER_SHOOTING_SPEED)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withPosition(3,1)
                .withSize(2, 1)
                .getEntry();
    dashboardEntryTopFeederIntakingSpeedSetting = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.TOP_FEEDER_INTAKING_OUTPUT_SETTING_NAME, 
                Constants.Feeder.Motors.TOP_FEEDER_INTAKING_SPEED)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withPosition(5,0)
                .withSize(2, 1)
                .getEntry();
    dashboardEntryBottomFeederIntakingSpeedSetting = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.BOTTOM_FEEDER_INTAKING_OUTPUT_SETTING_NAME,
                Constants.Feeder.Motors.BOTTOM_FEEDER_INTAKING_SPEED)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withPosition(5,1)
                .withSize(2, 1)
                .getEntry();
    // Limit switch
    dashboardEntryFeederNoteLimit = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.FEEDER_NOTE_LIMIT_NAME,
                false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(5,2)
                .withSize(1, 1)
                .getEntry();
    // Intake widgets
    dashboardEntryRightIntakeSpeed = dashboardTabUtility.add(
                  Constants.Dashboard.Utility.Widgets.RIGHT_INTAKE_OUTPUT_SETTING_NAME, 
                  Constants.Intake.Motors.RIGHT_INTAKE_SPEED_MAX)
                  .withWidget(BuiltInWidgets.kNumberSlider)
                  .withProperties(Map.of("min", 0, "max", 1))
                  .withPosition(7,0)
                  .withSize(2, 1)
                  .getEntry();
    // Arm widgets
    dashboardEntryArmVelocitySetting = dashboardTabUtility.add(
                  Constants.Dashboard.Utility.Widgets.ARM_VELOCITY_SETTING_NAME, 
                  Constants.Arm.Motors.ARM_VELOCITY)
                  .withWidget(BuiltInWidgets.kNumberSlider)
                  .withProperties(Map.of("min", 0, "max", Constants.Arm.Motors.ARM_VELOCITY_MAX))
                  .withPosition(7,1)
                  .withSize(2, 1)
                  .getEntry();
    dashboardEntryArmPositionSetting = dashboardTabUtility.add(
                  Constants.Dashboard.Utility.Widgets.ARM_POSITION_SETTING_NAME, 
                  Constants.Arm.SetPoints.HOME)
                  .withWidget(BuiltInWidgets.kNumberSlider)
                  .withProperties(Map.of("min", 0, "max", Constants.Arm.SetPoints.AMP))
                  .withPosition(7,2)
                  .withSize(2, 1)
                  .getEntry();
    dashboardEntryArmPosition = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.ARM_POSITION_NAME, 
              0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(2,2)
              .withSize(1, 1)
              .getEntry();
    // extra
    dashboardEntryPhantom = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.PHANTOM_NAME, 
              0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(0,3)
              .withSize(1, 1)
              .getEntry();
  }

  // Shooter gets
  public double getTopShooterVelocityDashboard() {
    return dashboardEntryTopShooterVelocitySetting.getDouble(0);
  }

  public double getBottomShooterVelocityDashboard() {
    return dashboardEntryBottomShooterVelocitySetting.getDouble(0);
  }

  // Feeder gets
  public double getTopFeederShootingSpeedDashboard() {
    return dashboardEntryTopFeederShootingSpeedSetting.getDouble(0);
  }

  public double getBottomFeederShootingSpeedDashboard() {
    return dashboardEntryBottomFeederShootingSpeedSetting.getDouble(0);
  }

  public double getTopFeederIntakingSpeedDashboard() {
    return dashboardEntryTopFeederIntakingSpeedSetting.getDouble(0);
  }

  public double getBottomFeederIntakingSpeedDashboard() {
    return dashboardEntryBottomFeederIntakingSpeedSetting.getDouble(0);
  }

  // Intake gets
  public double getRightIntakeSpeedDashboard() {
    return dashboardEntryRightIntakeSpeed.getDouble(0);
  }

  // Arm gets
  public double getArmPositionSettingDashboard() {
    return dashboardEntryArmPositionSetting.getDouble(0);
  }

  @Override
  public void periodic() {

    dashboardEntryTopShooterVelocity.setDouble(sysShooter.getShooterVelocity(Constants.Shooter.Motors.TOP_SHOOTER_PORTID));
    dashboardEntryBottomShooterVelocity.setDouble(sysShooter.getShooterVelocity(Constants.Shooter.Motors.BOTTOM_SHOOTER_PORTID));

    dashboardEntryFeederNoteLimit.setBoolean(sysFeeder.getNoteLimit());

    dashboardEntryArmPosition.setDouble(sysArm.getArmPosition());

    // This method will be called once per scheduler run
  }
}
