/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Commands;

import org.ligerbots.robot.Subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GoToIntakePositionCommandGroup extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GoToIntakePositionCommandGroup() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new ElevatorPositionCommand(ElevatorPosition.INTAKE_DEPLOY));
    addSequential(new IntakeDeployCommand());
    //TODO make a move wrist to level position command
    addSequential(new ElevatorPositionCommand(ElevatorPosition.BALL_INTAKE));
    addSequential(new GrabberToggleCommand(true));
  }
}
