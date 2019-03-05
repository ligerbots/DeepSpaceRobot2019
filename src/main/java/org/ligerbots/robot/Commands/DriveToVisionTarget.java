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
  double angle;                              //angle from robot to target in degrees (NT is initially in radians)
  double deltaAngle;
  double distanceToStrafe;

  boolean quit;
  boolean parallel = false;
  boolean angleFound = false;

  double parallelAngle;

  boolean setTurnControl = false;

  public DriveToVisionTarget() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencRies
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

    if (!parallel) {
      if (!angleFound) {
        findAngle();
        angleFound = true;
        Robot.driveTrain.enableTurningControl(parallelAngle - Robot.driveTrain.getYaw(), 0.5);
      }
      Robot.driveTrain.allDrive(0, Robot.driveTrain.getTurnOutput(), 0);

      if (Math.abs(Robot.driveTrain.getYaw() - parallelAngle) < 0.5) {
        parallel = true;
      }
    }
    else {
      if (!setTurnControl) {
        Robot.driveTrain.enableTurningControl(0, 0.5);
        setTurnControl = true;
      }
      visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
      distance = visionInfo[3];                                                 //reset distance and angle
      angle = visionInfo[4] * (180/Math.PI);
      deltaAngle = angle + (visionInfo[5] * (
        180/Math.PI));
      distanceToStrafe = Math.sin(visionInfo[4]) * distance;
      System.out.println(("Strafe Dist: " + distanceToStrafe + ", Angle 1: " + angle + ", Angle 2: " + visionInfo[5] * (180.0/Math.PI)));
      System.out.println("Yaw: " + Robot.driveTrain.getYaw());

      System.out.println("Dist: " + distance);
      /*if (distance > 60.0) {
        //Robot.driveTrain.allDrive(-0.5, Robot.driveTrain.turnSpeedCalc(angle), 0); 
      }
      else {*/
        Robot.driveTrain.allDrive(-Robot.driveTrain.driveSpeedCalc(distance), Robot.driveTrain.getTurnOutput(), Robot.driveTrain.strafeSpeedCalc(distanceToStrafe));
    // }
      //System.out.println("Angle 4: " + visionInfo[4] * 180 / Math.PI + ", Angle 5: " + visionInfo[5] * 180 / Math.PI);

      //Robot.driveTrain.allDrive(-Robot.driveTrain.driveSpeedCalc(distance), /*Robot.driveTrain.turnSpeedCalc(deltaAngle)*/0, Robot.driveTrain.strafeSpeedCalc(angle));
    }
    if (Math.abs(Robot.oi.getThrottle()) > 0.2) {
      quit = true;
    }
  }

  protected void findAngle () {
    double temp;
    double closest = 404;
    double best = 404;
    for (double angle : FieldMap.angles) {
      temp = Robot.driveTrain.getYaw() - angle;
      temp = Math.abs((temp + 180.0) % 360.0 - 180.0);
      if (temp < closest) {
        closest = temp;
        best = angle;
      }
    }
    System.out.println("Yaw: " + Robot.driveTrain.getYaw() + ", Picked Angle: " + best);
    parallelAngle = best;
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
