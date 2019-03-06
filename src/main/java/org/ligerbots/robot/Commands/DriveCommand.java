/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;

public class DriveCommand extends Command {
  private double savedYaw;
  double correctedYaw;

  public DriveCommand() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("running", "yes");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    /*if (Robot.oi.getStrafe() > 0) {//robot is strafing
      if (Math.abs(Robot.oi.getRotate()) > 0.04) {
        savedYaw = Robot.driveTrain.getYaw();
      }

      if (Math.abs(correctedYaw) > RobotMap.YAW_ERROR_THRESHOLD) {
        Robot.driveTrain.enableTurningControl(savedYaw - Robot.driveTrain.getYaw(), 0.3);

        correctedYaw = Robot.driveTrain.getTurnOutput();
      }
    } else {
      correctedYaw = Robot.oi.getRotate();
    }*/

    SmartDashboard.putString("LeftLeaderInfo", Robot.driveTrain.leftLeaderInfo());
    Robot.driveTrain.allDrive(Robot.oi.getThrottle(), /*correctedYaw*/Robot.oi.getRotate(), Robot.oi.getStrafe());
   // System.out.printf("Throttle: %5.2f, Rotate: %5.2f, Strafe: %5.2f", Robot.oi.getThrottle(), Robot.oi.getRotate(), Robot.oi.getStrafe());
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
