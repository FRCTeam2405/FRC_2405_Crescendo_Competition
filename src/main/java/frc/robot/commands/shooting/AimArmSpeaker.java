// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AimArmSpeaker extends Command {

  Arm arm;
  SwerveContainer swerveDrive;

  /** Creates a new AimArmSpeaker. */
  public AimArmSpeaker(SwerveContainer swerveDrive, Arm arm) {
    this.arm = arm;
    this.swerveDrive = swerveDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isEmpty()) {
      return;
    }

    Pose2d currentPose = swerveDrive.getPose();

    // Relative positions from the robot to the speaker
    double offsetX;
    double offsetY;
    double offsetZ;

    // Calculate XY offset between robot and speaker
    // and the angles between them
    if(alliance.get() == Alliance.Blue) {
      offsetX = Constants.Field.BLUE_SPEAKER_X - currentPose.getX();
      offsetY = Constants.Field.BLUE_SPEAKER_Y - currentPose.getY();
      offsetZ = Constants.Field.BLUE_SPEAKER_Z - Constants.Shooter.SHOOTER_HEIGHT;
    } else {
      offsetX = Constants.Field.RED_SPEAKER_X - currentPose.getX();
      offsetY = Constants.Field.RED_SPEAKER_Y - currentPose.getY();
      offsetZ = Constants.Field.RED_SPEAKER_Z - Constants.Shooter.SHOOTER_HEIGHT;
    }

    // calculate distances to the speaker for shooter calculations
    double floorDistance = Math.hypot(offsetX, offsetY);
    // double directDistance = Math.hypot(floorDistance, offsetZ);

    // linear interpolate between measured points to create
    // an angle for the arm to aim at.
    double desiredSetpoint;
    if(floorDistance < Constants.Arm.DynamicSetPoints.POINT_1) {
      desiredSetpoint = floorDistance * Constants.Arm.DynamicSetPoints.PIECE_0_COEFFICIENT + Constants.Arm.DynamicSetPoints.PIECE_0_CONSTANT;
    } else if(floorDistance < Constants.Arm.DynamicSetPoints.POINT_2) {
      desiredSetpoint = floorDistance * Constants.Arm.DynamicSetPoints.PIECE_1_COEFFICIENT + Constants.Arm.DynamicSetPoints.PIECE_1_CONSTANT;
    } else if(floorDistance < Constants.Arm.DynamicSetPoints.POINT_3) {
      desiredSetpoint = floorDistance * Constants.Arm.DynamicSetPoints.PIECE_2_COEFFICIENT + Constants.Arm.DynamicSetPoints.PIECE_2_CONSTANT;
    } else if(floorDistance < Constants.Arm.DynamicSetPoints.POINT_4) {
      desiredSetpoint = floorDistance * Constants.Arm.DynamicSetPoints.PIECE_3_COEFFICIENT + Constants.Arm.DynamicSetPoints.PIECE_3_CONSTANT;
    } else {
      desiredSetpoint = floorDistance * Constants.Arm.DynamicSetPoints.PIECE_4_COEFFICIENT + Constants.Arm.DynamicSetPoints.PIECE_4_CONSTANT;
    }

    // keep the setpoint within a safe range
    MathUtil.clamp(desiredSetpoint, 0, 37);

    arm.moveArmToPosition(desiredSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
