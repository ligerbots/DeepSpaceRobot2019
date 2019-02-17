/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.ligerbots.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX leader1;
  WPI_TalonSRX follower1;
  WPI_TalonSRX follower2;
  WPI_TalonSRX follower3;

  WPI_TalonSRX wrist;

  public double minHeight;

  PIDController pidController;

  public enum ElevatorPosition {
    HATCH_HIGH, HATCH_MID, HATCH_LOW, BALL_HIGH, BALL_MID, BALL_LOW, BALL_CARGO, BALL_INTAKE
  }

  public enum WristPosition {
    HIGH, FLAT, INTAKE
  }

  public Elevator () {

    leader1 = new WPI_TalonSRX(9); //It is the top left motor when looking at the back of the robot
    follower1 = new WPI_TalonSRX(8); //Same side as leader
    follower2 = new WPI_TalonSRX(7); //top right
    follower3 = new WPI_TalonSRX(6); //bottom right
    wrist = new WPI_TalonSRX(11);


    leader1.setInverted(true);
    follower1.setInverted(InvertType.FollowMaster);
    follower2.setInverted(InvertType.OpposeMaster);
    follower3.setInverted(InvertType.OpposeMaster);

    leader1.setSelectedSensorPosition(0);

    leader1.config_kP(0, 0.075);

    Arrays.asList(leader1, follower1, follower2, follower3, wrist)
        .forEach((WPI_TalonSRX talon) -> talon.enableCurrentLimit(true));

    Arrays.asList(leader1, follower1, follower2, follower3, wrist)
        .forEach((WPI_TalonSRX talon) -> talon.configContinuousCurrentLimit(8));
    
    Arrays.asList(leader1, follower1, follower2, follower3, wrist)
        .forEach((WPI_TalonSRX talon) -> talon.configPeakCurrentLimit(10));

    Arrays.asList(leader1, follower1, follower2, follower3, wrist)
        .forEach((WPI_TalonSRX talon) -> talon.configPeakCurrentDuration(3));

    Arrays.asList(leader1, follower1, follower2, follower3, wrist)
        .forEach((WPI_TalonSRX talon) -> talon.setNeutralMode(NeutralMode.Brake));

    Arrays.asList(follower1, follower2, follower3)
        .forEach((WPI_TalonSRX talon) -> talon.set(ControlMode.Follower, leader1.getDeviceID()));
  }


  public double getPosition () {
    return leader1.getSelectedSensorPosition() / RobotMap.ELEVATOR_ENCODER_TICKS_PER_REV * (Math.PI * 0.6) * (18.0 / 54.0); //I think the shaft is 0.5 inch diameter
  }

  public void setElevatorPID (double p, double i, double d) {
    leader1.config_kP(0, p);
    leader1.config_kI(0, i);
    leader1.config_kD(0, d);
  }

  public void set (double speed) {
    leader1.set(ControlMode.PercentOutput, speed);
  }

  public void setWristPID (double p, double i, double d) {
    wrist.config_kP(0, p);
    wrist.config_kI(0, i);
    wrist.config_kD(0, d);
  }

  public double getWristPosition () {
    return wrist.getSelectedSensorPosition() / RobotMap.WRIST_ENCODER_TICKS_PER_REV * (Math.PI * 0.5); //Still don't know shaft diameter 
  }

  public void setWrist (double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }

  public void setWristPosition (WristPosition pos) {
    switch (pos) {
      case HIGH:
        wrist.set(ControlMode.Position, 0.0); //FIX POSITIONS LATER
        break;
      case FLAT:
        wrist.set(ControlMode.Position, 0.0);
        break;
      case INTAKE:
        wrist.set(ControlMode.Position, 0.0);
        break;
    }
  }

  public void setElevatorPosition (ElevatorPosition pos) {
    switch (pos) {
      case HATCH_HIGH:
        leader1.set(ControlMode.Position, 59.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT); 
        break;
      case HATCH_MID:
        leader1.set(ControlMode.Position, 43.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        break;
      case HATCH_LOW:
        leader1.set(ControlMode.Position, 15.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        break;
      case BALL_CARGO:
        leader1.set(ControlMode.Position, 37.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        break;
      case BALL_INTAKE:
        leader1.set(ControlMode.Position, 0.0);
        break;
      case BALL_HIGH:
        leader1.set(ControlMode.Position, 59.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        break;
      case BALL_MID:
        leader1.set(ControlMode.Position, 51.5 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT); //temporary!!!!!
        break;
      case BALL_LOW:
        leader1.set(ControlMode.Position, 23.5 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        break;
    }
  }


  public int getClosedLoopError () {
    return leader1.getClosedLoopError();
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
