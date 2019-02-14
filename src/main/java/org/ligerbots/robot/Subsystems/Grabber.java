/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import org.ligerbots.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid armSolenoid;
  private DoubleSolenoid kickerSolenoid;
  boolean open = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Grabber(){
    kickerSolenoid = new DoubleSolenoid(RobotMap.PCM_ID, 1, 0); //placeholder channel numbers
    armSolenoid = new DoubleSolenoid(RobotMap.PCM_ID, 3, 2); //placeholder channel numbers
  }

  public void setPistons(boolean open){
    if (open){
      armSolenoid.set(Value.kReverse);
    }
    else{
      armSolenoid.set(Value.kForward);
    }
  }

  public void kicker() {
    if (open){
      kickerSolenoid.set(Value.kReverse);
      open = false;
    }
    else{
      kickerSolenoid.set(Value.kForward);
      Timer.delay(0.1);
      armSolenoid.set(Value.kReverse);
      open = true;
    }
  }
}
