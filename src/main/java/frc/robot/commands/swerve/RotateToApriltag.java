// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveContainer;
import swervelib.SwerveController;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class RotateToApriltag extends Command {

  double desiredPose;
  SwerveContainer swerveDrive;
  Limelight limelight;
  private DoubleSupplier moveX, moveY, turnTheta;

  /** Beta command for aiming at an apriltag. */
  public RotateToApriltag(SwerveContainer swerveDrive, Limelight limelight, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotate) {
  this.swerveDrive = swerveDrive;
  this.limelight = limelight;

  moveX = vX;
  moveY = vY;
  turnTheta = rotate;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDrive.inner.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double correctedMoveX = Math.pow(moveX.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedMoveY = Math.pow(moveY.getAsDouble(), 3) * Constants.Swerve.MAX_SPEED;
    double correctedTurnTheta = turnTheta.getAsDouble() * Constants.Swerve.MAX_ANGULAR_SPEED;

    // Gets apriltag position, if the Limelight returns null (tag not found), return early
    desiredPose = -limelight.getTargetPose(7);
    if(desiredPose == 0) {
      ChassisSpeeds desiredSpeeds = swerveDrive.inner.swerveController.getRawTargetSpeeds(correctedMoveX, correctedMoveY, correctedTurnTheta);
      swerveDrive.inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);

    }
    else {
     SmartDashboard.putNumber("testDisiredPose", desiredPose);

     double desiredYaw = desiredPose;

     ChassisSpeeds desiredSpeeds = swerveDrive.inner.swerveController.getRawTargetSpeeds(
       correctedMoveX, correctedMoveY,
       swerveDrive.inner.getPose().getRotation().getRadians() + (desiredYaw * Math.PI/180),
       swerveDrive.inner.getPose().getRotation().getRadians()
     );
     swerveDrive.inner.drive(SwerveController.getTranslation2d(desiredSpeeds), desiredSpeeds.omegaRadiansPerSecond, true, false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.inner.setHeadingCorrection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
