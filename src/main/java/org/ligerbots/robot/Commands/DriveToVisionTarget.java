/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ligerbots.robot.FieldMap;
import org.ligerbots.robot.Robot;


public class DriveToVisionTarget extends Command {

  double[] visionInfo;                                          //holds info recieved from the Odroid through NT
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};      //empty array of values to be used for default value when fetching
  double[] previousInfo;

  double distance;                           //total distance (raw from NT) from robot to target
  double angleRobotToTarget;                              //angle from robot to target in degrees (NT is initially in radians)
  double angleTargetToRobot;
  double verticalDistanceToTarget;

  boolean quit;
  boolean visionTargetFound;

  double targetAngle;

  boolean setTurnControl;


  public DriveToVisionTarget() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencRies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   // System.out.println("STARTED DRIVETOVISIONTARGET COMMAND");
    Robot.driveTrain.setLEDRing(true);
    SmartDashboard.putString("vision/active_mode", "rrtarget");
    quit = false;
    visionTargetFound = false;
    Robot.driveTrain.resetTurnI();
    setTurnControl = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // Need to read vision/target_info to see if we acquired the vision target
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
    distance = visionInfo[3];                                                 //reset distance and angle

    // See if we found the target
    if (!visionTargetFound) {
      visionTargetFound = (distance > 2);
    }

    // If we haven't found the target, wait until next time
    if (visionTargetFound) {
      angleRobotToTarget = Math.toDegrees(visionInfo[4]);
      angleTargetToRobot = Math.toDegrees(visionInfo[5]);

      if (Math.abs(angleTargetToRobot) > 1) { //Check if we're in the centerline of the target
        verticalDistanceToTarget = Math.cos(visionInfo[5]) * distance - 10;
        targetAngle = Math.acos(verticalDistanceToTarget/distance);
      }
      else {
        targetAngle = angleRobotToTarget;
      }
      // Enable smoothing
      if (!setTurnControl) {
        Robot.driveTrain.enableTurningControl(angleRobotToTarget+angleTargetToRobot, 0.5);
        setTurnControl = true;
      }

      Robot.driveTrain.allDrive(-Robot.driveTrain.driveSpeedCalc(distance), Robot.driveTrain.turnSpeedCalc(targetAngle));

      quit = distance < 20.0 && distance > 2;
      if (Math.abs(Robot.oi.getThrottle()) > 0.2) {
        quit = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    
    return quit; //This isn't totally done...
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
   // System.out.println("COMMAND ENDED");
    Robot.driveTrain.setLEDRing(false);
    //SmartDashboard.putString("vision/active_mode", "driver_front");   FIX LATER
    Robot.grabber.setPistons(true);
    Robot.driveCommand.start();
    SmartDashboard.putString("vision/active_mode", "rrtarget");

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
