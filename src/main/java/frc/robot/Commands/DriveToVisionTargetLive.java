/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems.DriveTrain.DriveSide;


public class DriveToVisionTargetLive extends Command {

  enum Phase {
    DRIVE, DRIVE_AND_ALIGN, ALIGN
  }
  
  double startAngle;
  double angleOffset;
  double targetAngle;
  double currentOffset;
  float estimatedDistance;
  float traveledDistance;

  public DriveToVisionTargetLive(float estimatedDistance, double angleOffset) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.estimatedDistance = estimatedDistance;
    this.angleOffset = angleOffset;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetAngle = Robot.driveTrain.getYaw() + angleOffset;
    traveledDistance = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentOffset = targetAngle - Robot.driveTrain.getYaw();

    traveledDistance += (Robot.driveTrain.getEncoderDistance(DriveSide.LEFT) + Robot.driveTrain.getEncoderDistance(DriveSide.RIGHT))/2;

    Robot.driveTrain.allDrive(1, Math.signum(currentOffset), Math.signum(Math.tan(angleOffset)));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return traveledDistance <= estimatedDistance/1.5; //Not sure if this can be replaced with something else, ie, passing in an angle that updates in real time to the command?
  } // Command will need to be interrupted in order to update the angleoffset

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
