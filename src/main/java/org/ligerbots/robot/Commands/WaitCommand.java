/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import edu.wpi.first.wpilibj.command.Command;

public class WaitCommand extends Command {
  long timeout;
  long startTime;
  
  public WaitCommand(long timeout) {
    this.timeout = timeout;
  }

  protected void initialize() {
    startTime = System.nanoTime();
  }

  protected void execute() {
  }

  protected boolean isFinished() {
    return (System.nanoTime() - startTime) >= timeout;
  }

  protected void end() {
  }

  protected void interrupted() {
  }
}