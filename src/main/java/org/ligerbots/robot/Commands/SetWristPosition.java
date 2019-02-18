/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.Subsystems.Elevator.WristPosition;

import edu.wpi.first.wpilibj.command.Command;

public class SetWristPosition extends Command {
  public WristPosition wristPosition;
  public SetWristPosition(WristPosition wristPosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setInterruptible(true);
    this.wristPosition = wristPosition;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.setWristPosition(wristPosition);
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
