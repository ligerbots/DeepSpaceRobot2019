/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import java.awt.Robot;
import java.util.Arrays;

import javax.swing.text.StyleContext.SmallAttributeSet;

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

  //WPI_TalonSRX wrist;

  public double minHeight;

  PIDController pidController;
  
  public AnalogInput encoder;

  public enum ElevatorPosition {
    HATCH_HIGH, HATCH_MID, HATCH_LOW, BALL_HIGH, BALL_MID, BALL_LOW, BALL_CARGO, BALL_INTAKE, INTAKE_CLEARANCE, START
  }

  public enum WristPosition {
    HIGH, FLAT, INTAKE
  }

  public ElevatorPosition currentPosition = ElevatorPosition.START;

  public WristPosition currentWrist = WristPosition.FLAT;

  public double hatchHigh = 56.5;
  public double hatchMid = 32.0;
  public double hatchLow = 6.5;
  public double ballHigh = 60.5; //more like 57
  public double ballMid = 42.0; //more like 33-35
  public double ballLow = 16.0; //more like 11.2
  public double ballCargo = 37.5; //more like 34.5
  public double ballIntake = 2.5;

  public double wristFlat = 1.74; //1.74 for first robot
  public double wristHigh = 0.68; // 2 or something

  public Elevator () {
    SmartDashboard.putNumber("FlatWristVal", RobotMap.WRIST_FLAT_VAL);
    SmartDashboard.putNumber("WristP", RobotMap.WRIST_P);
    SmartDashboard.putNumber("WristI", RobotMap.WRIST_I);
    SmartDashboard.putNumber("WristD", RobotMap.WRIST_D);
    SmartDashboard.putNumber("WristF", RobotMap.WRIST_F);

    leader1 = new WPI_TalonSRX(9); //It is the top left motor when looking at the back of the robot
    follower1 = new WPI_TalonSRX(13); //Same side as leader should be 8 on first robot
    follower2 = new WPI_TalonSRX(7); //top right
    follower3 = new WPI_TalonSRX(6); //bottom right
    //wrist = new WPI_TalonSRX(11);

    leader1.setInverted(true); //should be inverted
    follower1.setInverted(InvertType.OpposeMaster); //should follow
    follower2.setInverted(InvertType.OpposeMaster); //Should oppose on first
    follower3.setInverted(InvertType.FollowMaster); //should oppose

    leader1.setSelectedSensorPosition(0);
    leader1.setSensorPhase(false); //might need to change?

    leader1.config_kP(0, 0.000001); //0.075 for main robot one

    Arrays.asList(leader1, follower1, follower2, follower3/*, wrist*/)
        .forEach((WPI_TalonSRX talon) -> talon.enableCurrentLimit(true));

    Arrays.asList(leader1, follower1, follower2, follower3/*, wrist*/)
        .forEach((WPI_TalonSRX talon) -> talon.configContinuousCurrentLimit(8));
    
    Arrays.asList(leader1, follower1, follower2, follower3/*, wrist*/)
        .forEach((WPI_TalonSRX talon) -> talon.configPeakCurrentLimit(10));

    Arrays.asList(leader1, follower1, follower2, follower3/*, wrist*/)
        .forEach((WPI_TalonSRX talon) -> talon.configPeakCurrentDuration(3));

    encoder = new AnalogInput(RobotMap.ABSOLUTE_ENCODER_CHANNEL);
    //pidController = new PIDController(0, 0, 0, 0, encoder, wrist);

    // pidController.setPercentTolerance(0);


    leader1.set(ControlMode.PercentOutput, 0);
    
    Arrays.asList(leader1, follower1, follower2, follower3/*, wrist*/)
        .forEach((WPI_TalonSRX talon) -> talon.setNeutralMode(NeutralMode.Brake));

    Arrays.asList(follower1, follower2, follower3)
        .forEach((WPI_TalonSRX talon) -> talon.set(ControlMode.Follower, leader1.getDeviceID()));


  }


  public double getPosition () {
    return leader1.getSelectedSensorPosition() / RobotMap.ELEVATOR_ENCODER_TICKS_PER_REV * (Math.PI * RobotMap.SHAFT_DIAMETER) * (18.0 / 54.0); //I think the shaft is 0.5 inch diameter
  }

  public void setElevatorPID (double p, double i, double d) {
    leader1.config_kP(0, p);
    leader1.config_kI(0, i);
    leader1.config_kD(0, d);
  }

  public void set (double speed) {
    leader1.set(ControlMode.PercentOutput, speed);
  }

  public double getRawPosition() {
    return leader1.getSelectedSensorPosition();
  }

 /* public void setWristPID (double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setF(f);
  }

  public double getWristPosition () {
    if (RobotMap.WRIST_USES_ABSOLUTE_ENCODER) {   
      int offset = RobotMap.ABSOLUTE_ENCODER_OFFSET;
      int value = encoder.getValue();
      int offsetValue = (value + offset) % 4096;
      double angle = (offsetValue / 4096.0) * 360.0;
      return angle * (Math.PI * RobotMap.SHAFT_DIAMETER); //Shaft diameter.
    } else return wrist.getSelectedSensorPosition() / RobotMap.WRIST_ENCODER_TICKS_PER_REV * (Math.PI * RobotMap.SHAFT_DIAMETER); //Still don't know shaft diameter 
  }
  public void setWrist (double speed) {
    wrist.set(ControlMode.PercentOutput, speed);
  }
  public void setWristPosition (WristPosition pos) {
    //Should be positive P!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    setWristPID(RobotMap.WRIST_P, 0, 0, 0);
    if (RobotMap.WRIST_USES_ABSOLUTE_ENCODER) {
      switch (pos) {
        case HIGH:
          pidController.setSetpoint(wristHigh); 
          pidController.enable();
          wrist.setNeutralMode(NeutralMode.Brake);
          currentWrist = WristPosition.HIGH;
          break;
        case FLAT:
          System.out.println("going to 2.06");
          pidController.disable();
          currentWrist= WristPosition.FLAT;
          wrist.set(ControlMode.PercentOutput, 0);
          wrist.setNeutralMode(NeutralMode.Coast);
          break;
        case INTAKE:
          currentWrist = WristPosition.INTAKE;
          pidController.setSetpoint(1.5);
          break;
      }
      //Avoid waviness of if/elses
      return;
    }
    //Otherwise if we're using the motor's encoder
    switch (pos) {
      case HIGH:
        wrist.set(ControlMode.Position, 4.049); //FIX POSITIONS LATER 2.0 on FIRST ROBOT
        break;
      case FLAT:
        wrist.set(ControlMode.Position, RobotMap.WRIST_FLAT_VAL); //SHOULD BE FLAT VALLLLLL
        break;
      case INTAKE:
        wrist.set(ControlMode.Position, 0.0);
        break;
    }
  }*/

  public void setElevatorPosition (ElevatorPosition pos) {
    switch (pos) {
      case HATCH_HIGH:
        currentPosition = ElevatorPosition.HATCH_HIGH;
        leader1.set(ControlMode.Position, hatchHigh * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT); 
        //setWristPosition(WristPosition.FLAT);
        break;
      case HATCH_MID:
        currentPosition = ElevatorPosition.HATCH_MID;
        leader1.set(ControlMode.Position, hatchMid * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.FLAT);
        break;
      case HATCH_LOW:
        currentPosition = ElevatorPosition.HATCH_LOW;
        leader1.set(ControlMode.Position, hatchLow * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.FLAT);
        break;
      case BALL_CARGO:
        currentPosition = ElevatorPosition.BALL_CARGO;      
        leader1.set(ControlMode.Position, ballCargo * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.FLAT);
        break;
      case BALL_INTAKE:
        currentPosition = ElevatorPosition.BALL_INTAKE;
        leader1.set(ControlMode.Position, ballIntake * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.FLAT);
        break;
      case BALL_HIGH:
        currentPosition = ElevatorPosition.BALL_HIGH;
        leader1.set(ControlMode.Position, ballHigh * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.HIGH);
        break;
      case BALL_MID:
        currentPosition = ElevatorPosition.BALL_MID;
        leader1.set(ControlMode.Position, ballMid * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT); //temporary!!!!!
        //setWristPosition(WristPosition.FLAT);
        break;
      case BALL_LOW:
        currentPosition = ElevatorPosition.BALL_LOW;
        leader1.set(ControlMode.Position, ballLow * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.FLAT); 
        break;
      case INTAKE_CLEARANCE:
        currentPosition = ElevatorPosition.INTAKE_CLEARANCE;
        leader1.set(ControlMode.Position, (RobotMap.INTAKE_IN_HEIGHT + 1.2) * RobotMap.TICKS_TO_HEIGHT_COEFFICIENT);
        //setWristPosition(WristPosition.FLAT);
        break;
      case START:
      default:
        System.out.println("Huh?");
        break;
    }
  }


  public double getClosedLoopError () {
    return leader1.getClosedLoopError() / RobotMap.ELEVATOR_ENCODER_TICKS_PER_REV * (Math.PI * RobotMap.SHAFT_DIAMETER) * (18.0 / 54.0);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /*public void moveWristUp(double speed){
    wrist.set(ControlMode.PercentOutput, speed);
  }*/
}
