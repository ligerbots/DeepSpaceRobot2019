/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import org.ligerbots.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Compressor compressor;
  AnalogInput pressureGauge;

  public Pneumatics () {
    compressor = new Compressor(RobotMap.PCM_ID);
    compressor.setClosedLoopControl(false);
    pressureGauge = new AnalogInput(0);
  }

  public void setCompressor(boolean on) {
    compressor.setClosedLoopControl(on);
  }

  public double getPressure() {
    return 250.0 * (pressureGauge.getVoltage() / 5.2) - 25.0;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
