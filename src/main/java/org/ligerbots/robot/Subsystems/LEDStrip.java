/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import org.ligerbots.robot.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class LEDStrip extends Subsystem {
  //http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  
	int lastRow = 0;
	double voltage = -0.85;		// start at row 1

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    Spark revBlinkin = new Spark(RobotMap.LED_PWM_CHANNEL);
    public void doNothing () {
    	return;
    }
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setLights(int row) {
      // "row" is the row in the rev Robotics table of voltage to LED pattern
      voltage = -1.01+(0.02*row);
      if (row != lastRow) {
          System.out.println("Changed revblinkin speed " + row + "Voltage: " + voltage);
          lastRow = row;
      }
      revBlinkin.set(voltage);
      SmartDashboard.putNumber("LED Strip voltage", voltage);
    }
    
    public void refreshLights() {
      voltage = SmartDashboard.getNumber("LED Strip voltage", -0.85);
        revBlinkin.set(voltage);
    } 
    
    public void lightsOff() {
      revBlinkin.setSpeed(0.0);
    }

    public void setAllianceColor(DriverStation.Alliance color){
      if (color == Alliance.Red){
        setLights(81);
      } else if (color == Alliance.Blue){
        setLights(94);
      } else{
        setLights(6);
      }
    }
}
