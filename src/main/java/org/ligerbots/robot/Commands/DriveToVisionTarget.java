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


public class DriveToVisionTarget extends Command {

  double[] visionInfo;                                          //holds info recieved from the Odroid through NT
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};      //empty array of values to be used for default value when fetching
  double[] previousInfo;

  double distance;                           //total distance (raw from NT) from robot to target
  double angle;                              //angle from robot to target in degrees (NT is initially in radians)
  double deltaAngle;
  double[] lastVisionInfo;

  public DriveToVisionTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putString("vision/active_mode", "rrtarget");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
    distance = visionInfo[3];                                                 //reset distance and angle
    angle = visionInfo[4] * (180/Math.PI);
    deltaAngle = angle + (visionInfo[5] * (180/Math.PI));
    if (visionInfo[1] < 0.3 || ((Math.abs(distance - lastVisionInfo[3]) > 20 || Math.abs(angle - (lastVisionInfo[4] * 180 / Math.PI)) > 30) && lastVisionInfo[1] > 0.5)){
      return;
    }

    Robot.driveTrain.allDrive(Robot.driveTrain.driveSpeedCalc(distance), Robot.driveTrain.turnSpeedCalc(deltaAngle), Robot.driveTrain.strafeSpeedCalc(angle));
    lastVisionInfo = visionInfo;
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
