/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ligerbots.robot.FieldPosition;
import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;
import org.ligerbots.robot.Subsystems.DriveTrain.DriveSide;


public class DriveToVisionTarget extends Command {

  enum Phase {
    DRIVE, DRIVE_AND_ALIGN, ALIGN, FINISHED
  }
  
  double angleOffset;
  double targetAngle;
  double currentOffset;
  double alignAngle;
  double distanceToTarget;
  double distanceTraveled;
  FieldPosition targetPos;
  Phase currentPhase = Phase.DRIVE;

  public DriveToVisionTarget(double distanceToTarget, double angleOffset) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.distanceToTarget = distanceToTarget;
    this.angleOffset = angleOffset;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetAngle = Robot.driveTrain.getYaw() + angleOffset;
    SmartDashboard.putString("VisionTargetStatus", "DRIVE");
    SmartDashboard.putNumber("VisionTargetDistance", distanceToTarget);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentOffset = targetAngle - Robot.driveTrain.getYaw();
    distanceTraveled = (Robot.driveTrain.getEncoderDistance(DriveSide.RIGHT)+Robot.driveTrain.getEncoderDistance(DriveSide.LEFT))/2;
    distanceToTarget -= distanceTraveled;
    SmartDashboard.putNumber("VisionTargetCurrentAngleOffset", currentOffset);

    switch (currentPhase) {
      //Drives to most direct line to target, and rotates to orient along the line
      case DRIVE:
        //Will rotate to the given direction via PIDController, and strafe to the target
        Robot.driveTrain.enableTurningControl(currentOffset, 0.3);
        Robot.driveTrain.allDrive(Math.cos(angleOffset), Robot.driveTrain.getTurnOutput(), Math.sin(angleOffset));
        
        //Checking for distance threshold, whether we're close enough to start aligning
        if (distanceToTarget <= RobotMap.AUTO_DRIVE_STARTALIGN_THRESHOLD) {
          currentPhase = Phase.DRIVE_AND_ALIGN;
          SmartDashboard.putString("VisionTargetStatus", "DRIVE_AND_ALIGN");
        }
        
        break;
      //Drives along direct line to target while beginning to orient to the target
      case DRIVE_AND_ALIGN:
        //Checking for distance threshold, whether we're close enough to start aligning entirely
        if (Robot.driveTrain.getRobotPosition().distanceTo(targetPos) <= RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD) {
          currentPhase = Phase.ALIGN;
          SmartDashboard.putString("VisionTargetStatus", "ALIGN");
          break;
        }

        if (Math.abs(currentOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.driveTrain.enableTurningControl(currentOffset, 0.3);
          alignAngle = Robot.driveTrain.getTurnOutput(); //the smoothed value from the PIDController
        } else { //If the accuracy limit is reached, don't rotate at all
          alignAngle = 0;
        }

        //Full speed foward with strafing along a direct hypotenuse to the target
        Robot.driveTrain.allDrive(Math.cos(currentOffset), alignAngle, Math.sin(currentOffset));
        break;
      
      //STOPS foward motion, orients with target
      case ALIGN:
        if (Math.abs(currentOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.driveTrain.enableTurningControl(angleOffset, 0.3);
          alignAngle = Robot.driveTrain.getTurnOutput(); //the smoothed value from the PIDController
        } else {
          SmartDashboard.putString("VisionTargetStatus", "FINISHED");
          currentPhase = Phase.FINISHED;
          break;
        }

        //Will only rotate the robot to the specified offset
        Robot.driveTrain.allDrive(0, alignAngle, 0);

        break;
      default:
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
     //Will stop looping the command once we're finished aligning with the target
    return currentPhase == Phase.FINISHED;
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
