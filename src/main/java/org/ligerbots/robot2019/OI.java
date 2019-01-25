/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot2019;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * Add your docs here.
 */
public class OI {

    XboxController xbox;


    public OI () {
        xbox = new XboxController(0);
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
