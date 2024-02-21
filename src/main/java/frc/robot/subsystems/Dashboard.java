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

public class Dashboard extends SubsystemBase {

  private ShuffleboardTab dashboardTabMain, dashboardTabUtility;
  private GenericEntry  dashboardEntryTopShooterVelocity, dashboardEntryBottomShooterVelocity, 
                        dashboardEntryTopShooterVelocitySetting, dashboardEntryBottomShooterVelocitySetting,
                        dashboardEntryTopFeederSpeedSetting, dashboardEntryBottomFeederSpeedSetting,
                        dashboardEntryRightIntakeSpeed, dashboardEntryArmSpeed,
                        dashboardEntryArmPosition;

  /** Creates a new Dashboard. */
  public Dashboard() {

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
              .withSize(2, 1)
              .getEntry();
    dashboardEntryBottomShooterVelocity = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.BOTTOM_SHOOTER_VELOCITY_NAME, 
              0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(2,1)
              .withSize(2, 1)
              .getEntry();
    // Feeder widgets
    dashboardEntryTopFeederSpeedSetting = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.TOP_FEEDER_OUTPUT_SETTING_NAME, 
                Constants.Feeder.Motors.TOP_FEEDER_SPEED_MAX)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withPosition(5,0)
                .withSize(2, 1)
                .getEntry();
    dashboardEntryBottomFeederSpeedSetting = dashboardTabUtility.add(
                Constants.Dashboard.Utility.Widgets.BOTTOM_FEEDER_OUTPUT_SETTING_NAME,
                Constants.Feeder.Motors.BOTTOM_FEEDER_SPEED_MAX)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .withPosition(5,1)
                .withSize(2, 1)
                .getEntry();
    // Intake widgets
    dashboardEntryRightIntakeSpeed = dashboardTabUtility.add(
                  Constants.Dashboard.Utility.Widgets.RIGHT_INTAKE_OUTPUT_SETTING_NAME, 
                  Constants.Intake.Motors.RIGHT_INTAKE_SPEED_MAX)
                  .withWidget(BuiltInWidgets.kNumberSlider)
                  .withProperties(Map.of("min", 0, "max", 1))
                  .withPosition(7,2)
                  .withSize(2, 1)
                  .getEntry();
    // Arm widgets
    dashboardEntryArmSpeed = dashboardTabUtility.add(
                  Constants.Dashboard.Utility.Widgets.ARM_VELOCITY_SETTING_NAME, 
                  Constants.Arm.Motors.ARM_VELOCITY)
                  .withWidget(BuiltInWidgets.kNumberSlider)
                  .withProperties(Map.of("min", 0, "max", Constants.Arm.Motors.ARM_VELOCITY_MAX))
                  .withPosition(7,0)
                  .withSize(2, 1)
                  .getEntry();
    dashboardEntryArmPosition = dashboardTabUtility.add(
              Constants.Dashboard.Utility.Widgets.ARM_POSITION_NAME, 
              0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(7,1)
              .withSize(2, 1)
              .getEntry();
  }

  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }
}
