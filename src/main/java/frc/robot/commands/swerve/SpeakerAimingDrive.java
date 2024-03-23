// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveContainer;


public class SpeakerAimingDrive extends Command {

  Limelight limelight;
  SwerveContainer swerveDrive;
  Arm arm;
  Pose2d measuredPose;
  Pose2d currentPose;
  private DoubleSupplier moveX, moveY;
  double lastUpdateTime;
  Rotation2d desiredYaw;

  /** Drive command for aiming at the speaker while moving. 
   * measuredPose = pose measured from Limelight
   * currentPose = the pose the robot believes it is at
   * 
   * moveX = main driver x axis joystick inputs
   * moveY = main driver y axis joystick inputs
  */
  public SpeakerAimingDrive(Limelight limelight, SwerveContainer swerveDrive, Arm arm, DoubleSupplier vX, DoubleSupplier vY) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;
    this.arm = arm;

    measuredPose = limelight.getMeasuredPose();
    currentPose = swerveDrive.getPose();

    moveX = vX;
    moveY = vY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, swerveDrive, arm);
  }

  @Override
  public void initialize() {
    // allows the robot to use rotate to pose
    swerveDrive.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Don't aim the robot towards our speaker if we don't know what team we're on.
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isEmpty()) {
      return;
    }
    
    // Only check limelight once per second to avoid overloading network
    double timestamp = Timer.getFPGATimestamp() - limelight.getLatency();

    if (timestamp - lastUpdateTime >= 1) {
      measuredPose = limelight.getMeasuredPose();

      // Two tag measuring is stable, one tag measuring is not
      if(limelight.hasTarget() && limelight.tagCount() >= 2) {
        swerveDrive.addVisionMeasurement(new Pose2d(measuredPose.getX(), measuredPose.getY(), measuredPose.getRotation()), timestamp, VecBuilder.fill(0.01, 0.01, 999999999));
        lastUpdateTime = timestamp;
     }
    }

    currentPose = swerveDrive.getPose();

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
    } else {
      desiredSetpoint = floorDistance * Constants.Arm.DynamicSetPoints.PIECE_3_COEFFICIENT + Constants.Arm.DynamicSetPoints.PIECE_3_CONSTANT;
    }

    // keep the setpoint within a safe range
    MathUtil.clamp(desiredSetpoint, 0, 37);

    SmartDashboard.putNumber("floorDistance", floorDistance);
    SmartDashboard.putNumber("desiredSetpoint", desiredSetpoint);

    arm.moveArmToPosition(desiredSetpoint);

    // calculate angle to the speaker so we can aim that direction
    desiredYaw = new Rotation2d(offsetX, offsetY);
    desiredYaw.minus(Rotation2d.fromDegrees(90));

    SmartDashboard.putNumber("desiredYaw", desiredYaw.getDegrees());

    // normalizes input to [-MAX_SPEED, MAX_SPEED]
    double correctedMoveX = moveX.getAsDouble() * Constants.Swerve.MAX_SPEED;
    double correctedMoveY = moveY.getAsDouble() * Constants.Swerve.MAX_SPEED;

    // Invert inputs if we're on the red side of the field
    // so that movement is still relative to driver
    if(alliance.get() == Alliance.Red) {
      correctedMoveX *= -1;
      correctedMoveY *= -1;
    }
  
    // Rotation deadband so we don't oscillate
    //TODO! enable
    // if (Math.abs(currentPose.getRotation().getDegrees() - (desiredYaw.getDegrees() - 90)) > 0.25) {
    //   desiredYaw = swerveDrive.getYaw();
    // }

    swerveDrive.driveAbsolute(correctedMoveX, correctedMoveY, desiredYaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setHeadingCorrection(false);
  }

  // Returns true when the command should end during autonomous.
  @Override
  public boolean isFinished() {
    // return true;

    if (DriverStation.isAutonomousEnabled() && Math.abs(swerveDrive.getYaw().getDegrees() - desiredYaw.getDegrees()) > 0.25) {
      return true;
    } else {
      return false;
    }
  }
}
