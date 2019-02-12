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
import org.ligerbots.robot.Commands.ElevatorPositionCommand;
import org.ligerbots.robot.Commands.FieldCentricToggleCommand;
import org.ligerbots.robot.Commands.GrabberKickerToggleCommand;
import org.ligerbots.robot.Commands.GrabberToggleCommand;
import org.ligerbots.robot.Commands.IntakeToggleCommand;
import org.ligerbots.robot.Subsystems.Elevator.ElevatorPosition;


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
        xBoxBumperRight.whenPressed(new FieldCentricToggleCommand());

        JoystickButton xBoxBumperLeft = new JoystickButton(xbox, 5);
        xBoxBumperLeft.whenPressed(new IntakeToggleCommand());

        JoystickButton xBoxRightJoystick = new JoystickButton(xbox, 10);
        xBoxRightJoystick.whenPressed(new GrabberToggleCommand(true));

        JoystickButton xBoxLeftJoystick = new JoystickButton(xbox, 9);
        xBoxLeftJoystick.whenPressed(new GrabberToggleCommand(false));

        JoystickButton farmTwo = new JoystickButton(farm, 2);
        farmTwo.whenPressed(new ElevatorPositionCommand(ElevatorPosition.HATCH_LOW));

        JoystickButton farmThree = new JoystickButton(farm, 3);
        farmThree.whenPressed(new ElevatorPositionCommand(ElevatorPosition.HATCH_LOW));

        JoystickButton farmFour = new JoystickButton(farm, 4);
        farmFour.whenPressed(new ElevatorPositionCommand(ElevatorPosition.HATCH_MID));

        JoystickButton farmFive = new JoystickButton(farm, 5);
        farmFive.whenPressed(new ElevatorPositionCommand(ElevatorPosition.HATCH_HIGH));

        JoystickButton farmSix = new JoystickButton(farm, 6);
        farmSix.whenPressed(new ElevatorPositionCommand(ElevatorPosition.BALL_INTAKE));

        JoystickButton farmSeven = new JoystickButton(farm, 7);
        farmSeven.whenPressed(new ElevatorPositionCommand(ElevatorPosition.BALL_LOW));

        JoystickButton farmEight = new JoystickButton(farm, 8);
        farmEight.whenPressed(new ElevatorPositionCommand(ElevatorPosition.BALL_CARGO));

        JoystickButton farmNine = new JoystickButton(farm, 9);
        farmNine.whenPressed(new ElevatorPositionCommand(ElevatorPosition.BALL_MID));

        JoystickButton farmTen = new JoystickButton(farm, 10);
        farmTen.whenPressed(new ElevatorPositionCommand(ElevatorPosition.BALL_HIGH));

        JoystickButton farmSeventeen = new JoystickButton(farm, 17); //switch camera

        
        JoystickButton farmEighteen = new JoystickButton(farm, 18); //also switch camera
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

    public double getIntakeIn() {
        return xbox.getTriggerAxis(Hand.kRight);
    }

    public double getIntakeOut() {
        return xbox.getTriggerAxis(Hand.kLeft);
    }
}
