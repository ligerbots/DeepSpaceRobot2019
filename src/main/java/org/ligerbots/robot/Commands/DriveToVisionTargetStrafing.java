/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToVisionTargetStrafing extends Command {
  enum Phase{
    STRAFE_TO_ANGLE, DRIVE
  }
  double startAngle;
  org.ligerbots.robot.FieldPosition targetPos;
  double angleOffset;
  double targetAngle;
  double currentOffset;
  double currentDist;
  Phase currentPhase = Phase.STRAFE_TO_ANGLE;
  boolean finished;
  public DriveToVisionTargetStrafing(org.ligerbots.robot.FieldPosition targetPos, double angleOffset) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.targetPos = targetPos;
    this.angleOffset = angleOffset;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetAngle = Robot.driveTrain.getYaw() + angleOffset;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentOffset = targetAngle - Robot.driveTrain.getYaw();
    switch (currentPhase){
      case STRAFE_TO_ANGLE:
      //if (currentOffset > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD || currentDist < RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD){
        if (currentOffset < RobotMap.AUTO_TURN_ACCURACY_THRESHOLD){
          Robot.driveTrain.allDrive(0, 0, currentOffset/90);
          break;
        }else if (currentOffset > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD){
          Robot.driveTrain.allDrive(0, 0, -currentOffset/90);
          break;
        }
      currentPhase = Phase.DRIVE;
      break;
      case DRIVE:
      if (Robot.driveTrain.getRobotPosition().distanceTo(targetPos) > RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD) {
        Robot.driveTrain.allDrive(Robot.driveTrain.driveSpeedCalc(Robot.driveTrain.getRobotPosition().distanceTo(targetPos)), 0, 0);
        break;
      }
      finished = true;
      break;
      default:
      break;
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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
