/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import org.ligerbots.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;



public class TuneElevator extends Command {
  public TuneElevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  //goes up an inch a second at max speed
  @Override
  protected void execute() {
    switch (Robot.elevator.currentPosition) {
      case HATCH_HIGH:
        Robot.elevator.hatchHigh += Robot.oi.tuneElevator() / 50.0;
        break;
      case HATCH_MID:
        Robot.elevator.hatchMid += Robot.oi.tuneElevator() / 50.0;
        break;
      case HATCH_LOW:
        Robot.elevator.hatchLow += Robot.oi.tuneElevator() / 50.0;
        break;
      case BALL_CARGO:
        Robot.elevator.ballCargo += Robot.oi.tuneElevator() / 50.0;
        break;
      case BALL_INTAKE:
        Robot.elevator.ballIntake += Robot.oi.tuneElevator() / 50.0;
        break;
      case BALL_HIGH:
        Robot.elevator.ballHigh += Robot.oi.tuneElevator() / 50.0;
        break;
      case BALL_MID:
        Robot.elevator.ballMid += Robot.oi.tuneElevator() / 50.0;
        break;
      case BALL_LOW:
        Robot.elevator.ballLow += Robot.oi.tuneElevator() / 50.0;
        break;
      case INTAKE_CLEARANCE:
        break;
      case START:
        Robot.elevator.set(Robot.oi.tuneElevator() / 2);
      default:
        break;
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
