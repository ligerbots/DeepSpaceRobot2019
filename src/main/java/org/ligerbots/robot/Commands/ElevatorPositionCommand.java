/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;
import org.ligerbots.robot.Subsystems.Elevator.ElevatorPosition;
import org.ligerbots.robot.Subsystems.Elevator.WristPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorPositionCommand extends Command {

  ElevatorPosition pos;

  public ElevatorPositionCommand(ElevatorPosition pos) {
    this.pos = pos;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    SmartDashboard.putString("Elevator Place", pos.name());


   // Robot.intake.deployIntake(true);
    if (pos == ElevatorPosition.BALL_HIGH && Robot.elevator.currentPosition == ElevatorPosition.BALL_MID) Robot.elevator.setElevatorPID(0.1, 0.0, 0.0); 
    else Robot.elevator.setElevatorPID(0.075, 0.0, 0.0);

    Robot.elevator.setElevatorPosition(pos);

    /*if (pos == ElevatorPosition.BALL_HIGH) Robot.elevator.setWristPosition(WristPosition.HIGH);
    else if (pos == ElevatorPosition.BALL_INTAKE) Robot.elevator.setWristPosition(WristPosition.INTAKE);
    else Robot.elevator.setWristPosition(WristPosition.FLAT);*/
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute () {
    SmartDashboard.putNumber("Elevator Height", Robot.elevator.getPosition());
    SmartDashboard.putNumber("Elevator Error", Robot.elevator.getClosedLoopError());


    //Robot.elevator.setElevatorPosition(pos);
    if (pos != ElevatorPosition.BALL_INTAKE) {
      Robot.intake.deployIntake(false);
    }
    //if (Robot.elevator.getPosition() > RobotMap.INTAKE_IN_HEIGHT) Robot.intake.deployIntake(false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.intake.isIntakeDeployed() || pos == ElevatorPosition.BALL_INTAKE;
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
