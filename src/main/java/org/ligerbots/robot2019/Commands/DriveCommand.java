/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot2019.Commands;

import edu.wpi.first.wpilibj.command.Command;

public class DriveCommand extends Command {
  private double savedYaw = null;
  double correctedYaw;

  public DriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (oi.getStrafe() > 0) {//robot is strafing
      if (savedYaw == null) {
        savedYaw = driveTrain.getYaw();
      }

      if (Math.abs(correctedYaw) > RobotMap.YAW_ERROR_THRESHOLD) {
        driveTrain.enableTurningControl(savedYaw - driveTrain.getYaw(), 0.3);

        correctedYaw = driveTrain.getTurnOutput();
      }
    } else {
      savedYaw = null;
      correctedYaw = oi.getRotate();
    }

    robot.allDrive(oi.getThrottle(), correctedYaw, oi.getStrafe());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
