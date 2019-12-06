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


public class DriveToVisionTargetTest extends Command {

  public enum Phase {
    DRIVE, DRIVE_AND_ALIGN, ALIGN, FINISHED
  }
  
  double angleOffset;
  double targetAngle;
  double currentOffset;
  double alignAngle;
  double distanceTo;
  double time = 0;
  FieldPosition targetPos;
  Phase currentPhase = Phase.DRIVE;

  public DriveToVisionTargetTest(double angleOffset) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.angleOffset = angleOffset;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetAngle = Robot.driveTrain.getYaw() + angleOffset;
    SmartDashboard.putString("VisionTargetStatus", "DRIVE");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    time++;
    SmartDashboard.putNumber("TickElasped", time);

    switch (currentPhase) {
      //Drives to most direct line to target, and rotates to orient along the line
      case DRIVE:
        //Will rotate to the given direction via PIDController, and strafe to the target
        Robot.driveTrain.enableTurningControl(currentOffset, 0.3);
        Robot.driveTrain.allDrive(1, Robot.driveTrain.getTurnOutput(), Math.signum(Math.tan(angleOffset)));
        //If we've been driving for enough time, starting aligning and driving at the same time
        if (time >= 4) {
          currentPhase = Phase.DRIVE_AND_ALIGN;
          SmartDashboard.putString("VisionTargetStatus", "DRIVE_AND_ALIGN");
        }
        
        break;
      //Drives along direct line to target while beginning to orient to the target
      case DRIVE_AND_ALIGN:
        //If we've been driving for enough time, start aligning
        if (time >= 10) {
          currentPhase = Phase.ALIGN;
          SmartDashboard.putString("VisionTargetStatus", "ALIGN");
          break;
        }

        if (Math.abs(currentOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.driveTrain.enableTurningControl(currentOffset, 0.3);
          
          alignAngle = Robot.driveTrain.getTurnOutput(); //the smoothed value from the PIDController
          //TEST PURPOSES ONLY - WILL INCREMENTALLY REDUCE THE CURRENTOFFSET
          currentOffset = (currentOffset > 0) ? currentOffset-1 : (currentOffset < 0) ? currentOffset+1 : 0;
        } else { //If the accuracy limit is reached, don't rotate at all
          alignAngle = 0;
        }
        SmartDashboard.putNumber("AlignAngle", alignAngle);

        //Full speed foward with strafing along a direct hypotenuse to the target
        Robot.driveTrain.allDrive(1, alignAngle, Math.signum(Math.tan(currentOffset)*5));
        break;
      
      //STOPS foward motion, orients with target
      case ALIGN:
        if (Math.abs(currentOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.driveTrain.enableTurningControl(angleOffset, 0.3);
          alignAngle = Robot.driveTrain.getTurnOutput(); //the smoothed value from the PIDController
          //TEST PURPOSES ONLY - WILL INCREMENTALLY REDUCE THE CURRENTOFFSET
          currentOffset = (currentOffset > 0) ? currentOffset-1 : (currentOffset < 0) ? currentOffset+1 : 0;
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

    SmartDashboard.putNumber("AngleOffset", currentOffset);
    SmartDashboard.putNumber("ThrottleTurning", Math.signum(Math.tan(currentOffset)));
  }

  public DriveToVisionTargetTest.Phase GetCurrentPhase() {
    return currentPhase;
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