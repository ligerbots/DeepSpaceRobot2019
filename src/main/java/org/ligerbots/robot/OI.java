/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot;

import org.ligerbots.robot.Commands.DriveCommand;
import org.ligerbots.robot.Commands.ElevatorPositionCommand;
import org.ligerbots.robot.Subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Add your docs here.
 */
public class OI {

    XboxController xbox;


    public OI () {
        xbox = new XboxController(0);

        JoystickButton elevator = new JoystickButton(xbox, 1);
        elevator.whenPressed(new ElevatorPositionCommand(ElevatorPosition.BALL_MID));
    }

    public double getThrottle () {
        return xbox.getY(Hand.kLeft);
    }

    public double getRotate () {
        return xbox.getX(Hand.kRight);
    }

    public double getStrafe () {
        return xbox.getX(Hand.kLeft);
    }

    public double elevatorUp () {
        return xbox.getTriggerAxis(Hand.kRight);
    }

    public double elevatorDown () {
        return xbox.getTriggerAxis(Hand.kLeft);
    }
}
