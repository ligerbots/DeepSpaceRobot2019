/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import org.ligerbots.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewScore extends Command {
  double[] visionInfo;                                          //holds info recieved from the Odroid through NT
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};      //empty array of values to be used for default value when fetching
  double[] previousInfo;

  double distance;                           //total distance (raw from NT) from robot to target
  double angle;                              //angle from robot to target in degrees (NT is initially in radians)
  double deltaAngle;
  double distanceToStrafe;
  double angle2;

  boolean quit;
  boolean parallel;
  boolean visionTargetFound;

  double parallelAngle;

  boolean setTurnControl;

  enum CommandState {LINE_UP, DRIVE_IN};

  CommandState commandState;


  public NewScore() {
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
    commandState = CommandState.LINE_UP;
    quit = false;
    parallel = false;
    visionTargetFound = false;
    Robot.driveTrain.resetTurnI();
    setTurnControl = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    System.out.println("Current Mode: " + (commandState == CommandState.LINE_UP ? "Line Up" : "Deliver"));
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
    distance = visionInfo[3]; //reset distance and angle

    // See if we found the target
    if (!visionTargetFound) {
      visionTargetFound = (distance > 2);
    }

    // If we haven't found the target, wait until next time
    if (visionTargetFound) {
      angle = visionInfo[4] * (180/Math.PI);
      deltaAngle = angle + (visionInfo[5] * (180/Math.PI));
      distanceToStrafe = Math.sin(visionInfo[5]) * distance;
      angle2 = visionInfo[5] * (180.0 / Math.PI);
      if (!setTurnControl) {
        Robot.driveTrain.enableTurningControl(deltaAngle, 0.1);
        setTurnControl = true;
      }
      switch (commandState) {
        case LINE_UP:
          Robot.driveTrain.allDrive(/*-Robot.driveTrain.driveSpeedCalcPlace(distance)*/0, Robot.driveTrain.turnSpeedCalc(angle), /*Robot.driveTrain.strafeSpeedCalc(distanceToStrafe)*/0);

          if (Math.abs(angle) < 3.0/* && distance < 30*/) {
            commandState = CommandState.DRIVE_IN;
          }
          break;

        case DRIVE_IN:
          Robot.driveTrain.allDrive(-0.4, Robot.driveTrain.turnSpeedCalc(angle), 0);
          break;
    }

      System.out.println("Angle 1: " + angle + ", Angle 2: " + visionInfo[5] * (180.0 / Math.PI));

    }
  }

 /* protected void findAngle () {
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
   // System.out.println("Yaw: " + Robot.driveTrain.getYaw() + ", Picked Angle: " + best);
    parallelAngle = best;
  }*/

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    System.out.println("Dist: " + distance);
    return (Math.abs(Robot.oi.getThrottle()) > 0.2) || (distance < 20.0 && distance > 0.1); 

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("COMMAND ENDED");
    Robot.driveTrain.setLEDRing(false);
    //SmartDashboard.putString("vision/active_mode", "driver_front");   FIX LATER
    //
    //Robot.grabber.setPistons(true);
    Robot.driveCommand.start();
    SmartDashboard.putString("vision/active_mode", "driver_target");

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
