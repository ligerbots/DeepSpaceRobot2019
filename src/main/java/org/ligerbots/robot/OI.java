/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot;

import org.ligerbots.robot.Commands.CompressorCommand;
import org.ligerbots.robot.Commands.DriveCommand;
import org.ligerbots.robot.Commands.ElevatorPositionCommand;
import org.ligerbots.robot.Commands.IntakeRunCommand;
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
        elevator.whenPressed(new ElevatorPositionCommand(ElevatorPosition.HATCH_HIGH));

        JoystickButton compressorOff = new JoystickButton (xbox, 7);
        compressorOff.whenPressed(new CompressorCommand(false));

        JoystickButton compressorOn = new JoystickButton (xbox, 8);
        compressorOn.whenPressed(new CompressorCommand(true));

        JoystickButton rightBumper = new JoystickButton (xbox, 6);
        rightBumper.whileHeld(new IntakeRunCommand(true));

    }

    public double getThrottle () {
        return  Math.abs(xbox.getY(Hand.kLeft)) > 0.05 ? xbox.getY(Hand.kLeft) : 0.0;
    }

    public double getRotate () {
        return Math.abs(xbox.getX(Hand.kRight)) > 0.05 ? xbox.getX(Hand.kRight) : 0.0;
    }

    public double getStrafe () {
        return  Math.abs(xbox.getX(Hand.kLeft)) > 0.05 ? xbox.getX(Hand.kLeft) : 0.0;
    }

    public double elevatorUp () {
        return xbox.getTriggerAxis(Hand.kRight);
    }

    public double elevatorDown () {
        return xbox.getTriggerAxis(Hand.kLeft);
    }
}
