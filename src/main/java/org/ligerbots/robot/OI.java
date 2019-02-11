/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.ligerbots.robot.Commands.DriveToVisionTarget;
import org.ligerbots.robot.Commands.GrabberKickerToggleCommand;
import org.ligerbots.robot.Commands.GrabberToggleCommand;
import org.ligerbots.robot.Commands.IntakeToggleCommand;


/**
 * Add your docs here.
 */
public class OI {

    XboxController xbox;
    Joystick farm;


    public OI () {
        xbox = new XboxController(0);
        farm = new Joystick(1);

        JoystickButton xBoxA = new JoystickButton(xbox, 1);
        xBoxA.whenPressed(new GrabberKickerToggleCommand());

        JoystickButton xBoxB = new JoystickButton(xbox, 2);
        xBoxB.whenPressed(new DriveToVisionTarget());

        JoystickButton xBoxBumperRight = new JoystickButton(xbox, 6);
        //xBoxBumperRight.whenPressed(new FieldCentricToggleCommand()); TODO make field centric toggle command

        JoystickButton xBoxBumperLeft = new JoystickButton(xbox, 5);
        xBoxBumperLeft.whenPressed(new IntakeToggleCommand());

        JoystickButton xBoxRightJoystick = new JoystickButton(xbox, 10);
        xBoxRightJoystick.whenPressed(new GrabberToggleCommand(true));

        JoystickButton xBoxLeftJoystick = new JoystickButton(xbox, 9);
        xBoxLeftJoystick.whenPressed(new GrabberToggleCommand(false));
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
}
