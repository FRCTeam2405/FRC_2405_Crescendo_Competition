// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pixy2_library.Pixy2;
import frc.robot.pixy2_library.Pixy2.LinkType;
import frc.robot.pixy2_library.Pixy2CCC.Block;
import frc.robot.pixy2_library.links.Link;

public class PixyCam extends SubsystemBase {

  private Pixy2 pixy;

  /** Creates a new PixyCam. */
  public PixyCam() {
    pixy = Pixy2.createInstance(LinkType.SPI);
    this.initialize();
  }

  public void initialize() {
    pixy.init(4);
  }

  @Override
  public void periodic() {
    // Tell the Pixy2 to get the viewed blocks
    pixy.getCCC().getBlocks();
  }

  public ArrayList<Block> getBlocks() {
    return pixy.getCCC().getBlockCache();
  }
}
