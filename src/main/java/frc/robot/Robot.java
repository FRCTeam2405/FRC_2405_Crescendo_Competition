// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// IMPORTANT! - You may be looking for RobotContainer. No new code
// should be added here unless you know what you are doing.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    // Set up logging. One of the few things we manage in Robot.java
    if(isReal()) {
      // Log to NetworkTables and (optionally) a USB stick
      Logger.addDataReceiver(new NT4Publisher());
      // Logger.addDataReceiver(new WPILOGWriter());
    } else {
      // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    // Start AdvantageKit logger
    Logger.start();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // @Override
  // public void autonomousInit() {
  //   autonomousCommand = robotContainer.getAutonomousCommand();

  //   if (autonomousCommand != null) {
  //     autonomousCommand.schedule();
  //   }
  // }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  @Override
  public void disabledExit() {}
  @Override
  public void autonomousPeriodic() {}
  @Override
  public void autonomousExit() {}
  @Override
  public void teleopPeriodic() {}
  @Override
  public void teleopExit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void testExit() {}
}
