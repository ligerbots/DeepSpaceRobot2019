/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.FieldPosition;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class DriveToVisionTarget extends Command {

  enum Phase {
    DRIVE, DRIVE_AND_ALIGN, ALIGN
  }
  
  double startAngle;
  double angleOffset;
  double targetAngle;
  double currentOffset;
  double alignAngle;
  FieldPosition targetPos;
  Phase currentPhase = Phase.DRIVE;

  public DriveToVisionTarget(FieldPosition targetPos, double angleOffset) {
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

    switch (currentPhase) {
      case DRIVE:
        Robot.driveTrain.allDrive(1, Math.signum(currentOffset), Math.signum(Math.tan(angleOffset)));
        
        if (Robot.driveTrain.getRobotPosition().distanceTo(targetPos) <= RobotMap.AUTO_DRIVE_STARTALIGN_THRESHOLD) {
          currentPhase = Phase.DRIVE_AND_ALIGN;
        }
        
        break;

      case DRIVE_AND_ALIGN:
        if (Math.abs(currentOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.driveTrain.enableTurningControl(currentOffset, 0.3);

          alignAngle = Robot.driveTrain.getTurnOutput(); //the smoothed value from the PIDController
        }

        Robot.driveTrain.allDrive(1, alignAngle, Math.signum(Math.tan(angleOffset)));

        if (Robot.driveTrain.getRobotPosition().distanceTo(targetPos) <= RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD) {
          currentPhase = Phase.ALIGN;
        }
      break;

      case ALIGN:
        if (Math.abs(currentOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.driveTrain.enableTurningControl(angleOffset, 0.3);

          alignAngle = Robot.driveTrain.getTurnOutput(); //the smoothed value from the PIDController
        }

        Robot.driveTrain.allDrive(0, alignAngle, 0);

        break;
      default:
    }
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
