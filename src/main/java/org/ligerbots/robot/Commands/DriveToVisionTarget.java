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

  boolean quit;

  public DriveToVisionTarget() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("STARTED DRIVETOVISIONTARGET COMMAND");
    Robot.driveTrain.setLEDRing(true);
    SmartDashboard.putString("vision/active_mode", "rrtarget");
    quit = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
    distance = visionInfo[3];                                                 //reset distance and angle
    angle = visionInfo[4] * (180/Math.PI);
    deltaAngle = angle + (visionInfo[5] * (180/Math.PI));

    System.out.println(Robot.driveTrain.strafeSpeedCalc(angle));

    Robot.driveTrain.allDrive(/*-Robot.driveTrain.driveSpeedCalc(distance)*/0, /*Robot.driveTrain.turnSpeedCalc(deltaAngle)*/0, Robot.driveTrain.strafeSpeedCalc(angle));
    if (Math.abs(Robot.oi.getThrottle()) > 0.2) {
      quit = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return /*distance <= 40 && Math.abs(angle) <= 1*/quit; //This isn't totally done...
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("COMMAND ENDED");
    //Robot.driveTrain.setLEDRing(false);
    //SmartDashboard.putString("vision/active_mode", "driver_front");   FIX LATER
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
