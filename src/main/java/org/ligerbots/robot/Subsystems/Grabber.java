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
  private DoubleSolenoid armsolenoid;
  private DoubleSolenoid kickersolenoid;
  boolean open = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Grabber(){
    kickersolenoid = new DoubleSolenoid(RobotMap.PCM_ID, 1, 0); //placeholder channel numbers
    armsolenoid = new DoubleSolenoid(RobotMap.PCM_ID, 3, 2); //placeholder channel numbers
  }

  public void setPistons(boolean open){
    if (open){
      armsolenoid.set(Value.kReverse);
    }
    else{
      armsolenoid.set(Value.kForward);
    }
  }

  public void kicker() {
    if (open){
      kickersolenoid.set(Value.kReverse);
      open = false;
    }
    else{
      kickersolenoid.set(Value.kForward);
      Timer.delay(0.1);
      armsolenoid.set(Value.kReverse);
      open = true;
    }
  }
}
