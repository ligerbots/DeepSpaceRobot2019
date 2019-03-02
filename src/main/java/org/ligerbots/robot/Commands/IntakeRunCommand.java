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

public class IntakeRunCommand extends Command {
  public IntakeRunCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("Intake Speed", 0.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // System.out.printf("In: %5.2f, Out: %5.2f", Robot.oi.getIntakeIn(), Robot.oi.getIntakeOut());
    Robot.intake.setIntakeMotor(Robot.oi.getIntakeIn() - Robot.oi.getIntakeOut()/*SmartDashboard.getNumber("Intake Speed", 0.0)*/);
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
