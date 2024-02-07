// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {}
  
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

  public void initialize(){
  networkTable.getEntry("ledMode").setNumber(0);
  networkTable.getEntry("pipeline").setNumber(4);
  }
  
  public double[] getRobotPose() {
  return networkTable.getEntry("botpose").getDoubleArray(new double[6]);
  }
  public double[] getTargetPose(int tid) {
    if(networkTable.getEntry("tid").getInteger(-1) != tid) {
      return null;
    }

    return networkTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
