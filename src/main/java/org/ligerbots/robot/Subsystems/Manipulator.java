/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.ligerbots.robot.RobotMap;
/**
 * Add your docs here.
 */
public class Manipulator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  DoubleSolenoid solenoid1;
  DoubleSolenoid solenoid2;
  DoubleSolenoid solenoid3;
  WPI_TalonSRX talon1;
  WPI_TalonSRX talon2;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setMotorPID(double p, double i, double d) {
      talon1.config_kP(0, p);
      talon1.config_kI(0, i);
      talon1.config_kD(0, d);
  }
  public void setMotorPosition(double setPoint);
  public void openSolenoids() {
    
  }
}
