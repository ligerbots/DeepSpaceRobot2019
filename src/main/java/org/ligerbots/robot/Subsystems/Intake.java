/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  boolean isDeployed = true;
  boolean isStart = true;
  WPI_TalonSRX intakeMotor;
  DoubleSolenoid intakeSolenoid;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Intake(){
    intakeMotor = new WPI_TalonSRX(RobotMap.CT_INTAKE);
    intakeSolenoid = new DoubleSolenoid(RobotMap.PCM_ID, 1, 0); //placeholder channel numbers

    intakeMotor.configContinuousCurrentLimit(30);
    intakeMotor.configPeakCurrentLimit(35);
    intakeMotor.configPeakCurrentDuration(3);
  }

  public void toggleIntake(){
    if (isDeployed){
      intakeSolenoid.set(Value.kReverse);
      isDeployed = false;
    }
    else {
      intakeSolenoid.set(Value.kForward);
      isDeployed = true;
    }
  }

  public void deployIntake (boolean deploy) {
    if (Robot.elevator.getPosition() > RobotMap.INTAKE_IN_HEIGHT || isStart) {
      intakeSolenoid.set(deploy ? Value.kForward : Value.kReverse);
      isDeployed = deploy;
      isStart = false;
    }
  }
  //FIXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  public void setIntakeMotor(double value){
    /*if (isDeployed)*/ intakeMotor.set(ControlMode.PercentOutput, value);
  }
  public boolean isIntakeDeployed(){
    return isDeployed;
  }
}
