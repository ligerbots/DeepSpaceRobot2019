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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ligerbots.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
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

  public WPI_TalonSRX wrist;

  public double minHeight;

  public PIDController pidController;
  
  public AnalogInput encoder;

  public enum ElevatorPosition {
    HATCH_HIGH, HATCH_MID, HATCH_LOW, BALL_HIGH, BALL_MID, BALL_LOW, BALL_CARGO, BALL_INTAKE, INTAKE_DEPLOY, ONE
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

    if (RobotMap.WRIST_USES_ABSOLUTE_ENCODER) {
      encoder = new AnalogInput(RobotMap.ABSOLUTE_ENCODER_CHANNEL);
      pidController = new PIDController(0.0001, 0, 0, 0, encoder, wrist);
      pidController.enable();;

      leader1.set(ControlMode.PercentOutput, 0);
    }
    
    Arrays.asList(leader1, follower1, follower2, follower3, wrist)
        .forEach((WPI_TalonSRX talon) -> talon.setNeutralMode(NeutralMode.Brake));

    Arrays.asList(follower1, follower2, follower3)
        .forEach((WPI_TalonSRX talon) -> talon.set(ControlMode.Follower, leader1.getDeviceID()));
  }


  public double getPosition () {
    return leader1.getSelectedSensorPosition() / RobotMap.ELEVATOR_ENCODER_TICKS_PER_REV * (Math.PI * RobotMap.SHAFT_DIAMETER) * (18.0 / 54.0); //I think the shaft is 0.5 inch diameter
  }

  // This shouldn't be necessary since we're already setting kP above.
  // public void setElevatorPID (double p, double i, double d) {
  //   leader1.config_kP(0, p);
  //   leader1.config_kI(0, i);
  //   leader1.config_kD(0, d);
  // }

  public void set (double speed) {
    leader1.set(ControlMode.PercentOutput, speed);
  }

  public void resetEncoderZero () {
    leader1.setSelectedSensorPosition(0);
  }

  // This won't work. The wrist Talon can't do it's own PID since the encoder is connecte tot he Rio
  // public void setWristPID (double p, double i, double d) {
  //   wrist.config_kP(0, p);
  //   wrist.config_kI(0, i);
  //   wrist.config_kD(0, d);
  // 1230 horizontal
  // 1550 angled up
  // }

  public double getWristPosition () {
    if (RobotMap.WRIST_USES_ABSOLUTE_ENCODER) {   
      int value = encoder.getValue();
      int offsetValue = (value - RobotMap.ABSOLUTE_ENCODER_OFFSET) % 4096;
      double angle = (offsetValue / 4096.0) * 360.0;
      return angle;// no need to use shaft diameter * (Math.PI * RobotMap.SHAFT_DIAMETER); //Shaft diameter.
    } else return wrist.getSelectedSensorPosition() / RobotMap.WRIST_ENCODER_TICKS_PER_REV * (Math.PI * RobotMap.SHAFT_DIAMETER); //Still don't know shaft diameter 
  }

  public void setWrist (double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }

  public void setWristPosition (WristPosition pos) {
    if (RobotMap.WRIST_USES_ABSOLUTE_ENCODER) {
      System.out.println("wrist pos" + pos);
     
      switch (pos) {
        case HIGH:
          pidController.setSetpoint(1550); //FIX POSITIONS LATER
          break;
        case FLAT:
          pidController.setSetpoint(1230);
          break;
        case INTAKE:
          pidController.setSetpoint(1000);
          break;
      }
      //Avoid waviness of if/elses
      return;
    }
    //Otherwise if we're using the motor's encoder
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
        setWristPosition(WristPosition.FLAT);
        break;
      case HATCH_MID:
        leader1.set(ControlMode.Position, 43.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.FLAT);
        break;
      case HATCH_LOW:
        leader1.set(ControlMode.Position, 15.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.FLAT);
        break;
      case BALL_CARGO:
        leader1.set(ControlMode.Position, 37.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.HIGH);
        break;
      case BALL_INTAKE:
        leader1.set(ControlMode.Position, 0.0);
        setWristPosition(WristPosition.FLAT);
        break;
      case BALL_HIGH:
        leader1.set(ControlMode.Position, 59.0 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.HIGH);
        break;
      case BALL_MID:
        leader1.set(ControlMode.Position, 51.5 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT); //temporary!!!!!
        setWristPosition(WristPosition.FLAT);
        break;
      case BALL_LOW:
        leader1.set(ControlMode.Position, 23.5 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.FLAT);
        break;
      case INTAKE_DEPLOY:
        leader1.set(ControlMode.Position, 27 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.FLAT);
        break;
      case ONE:
        leader1.set(ControlMode.Position, 2 * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        setWristPosition(WristPosition.FLAT);
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

  public void moveWristUp(double speed){
    wrist.set(ControlMode.PercentOutput, speed);
  }
}
