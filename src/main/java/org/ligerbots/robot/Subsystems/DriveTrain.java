/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;


@SuppressWarnings("deprecation")
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  enum DriveSide {
    LEFT, RIGHT, CENTER
  }

  CANSparkMax leftLeader;
  CANSparkMax leftFollower;
  CANSparkMax rightLeader;
  CANSparkMax rightFollower;
  CANSparkMax centerLeader;
  CANSparkMax centerFollower;
  DifferentialDrive diffDrive;
  Boolean fieldCentric = false;
  PIDController turningController;
  double limitedThrottle;
  AHRS navX;

  double turnOutput;

  Relay spike;


  public DriveTrain () {

    leftLeader = new CANSparkMax(14, MotorType.kBrushless);
    leftFollower = new CANSparkMax(1, MotorType.kBrushless);
    rightLeader = new CANSparkMax(2, MotorType.kBrushless);
    rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    centerLeader = new CANSparkMax(4, MotorType.kBrushless);
    centerFollower = new CANSparkMax(5, MotorType.kBrushless);


    leftLeader.setInverted(false);
    leftFollower.follow(leftLeader, false);
    rightFollower.follow(rightLeader);
    centerFollower.follow(centerLeader); //MIGHT NEED TO BE INVERTED

    diffDrive = new DifferentialDrive(leftLeader, rightLeader); //fix l8r

    diffDrive.setSafetyEnabled(false);

    navX = new AHRS(Port.kMXP, (byte) 200);

    spike = new Relay(1); //is 1 on first robot

    centerLeader.setOpenLoopRampRate(0.005);
 //centerLeader.setSmartCurrentLimit(20);

    Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower, centerLeader, centerFollower)
         .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(40));

         Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower, centerLeader, centerFollower)
         .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));
    turningController = new PIDController(0.033, 0.000, 0.0, navX, output -> this.turnOutput = output);

    //centerLeader.setOpenLoopRampRate(0.3);

    rightLeader.setOpenLoopRampRate(0.0065);
    leftLeader.setOpenLoopRampRate(0.0065);
  }
  
  double squaredStrafe;
  double currentRampedStrafe = 0;
  double lastStrafe = 0;
  public void allDrive (double throttle, double rotate, double strafe) {
    squaredStrafe = strafe/2/* * strafe * Math.signum(strafe)*/;
    if (fieldCentric) {
      diffDrive.arcadeDrive(throttle * Math.cos(getYaw() + squaredStrafe * Math.sin(getYaw())), rotate);
      centerLeader.set(-throttle * Math.sin(getYaw()) + squaredStrafe * Math.cos(getYaw()));
    }
    else
    {
      if (Robot.elevator.getPosition() > 40) {
        limitedThrottle = Math.abs(throttle) > 0.5 ? 0.5 * Math.signum(throttle) : throttle;
      }
      else {
        limitedThrottle = throttle;
      }
      diffDrive.arcadeDrive(limitedThrottle, -rotate);

      currentRampedStrafe = lastStrafe + 0.01 * Math.signum(squaredStrafe);
      if (Math.abs(squaredStrafe) < Math.abs(currentRampedStrafe)) {
        currentRampedStrafe = squaredStrafe;
      }
      // System.out.println("Strafe Speed: " + currentRampedStrafe);
      centerLeader.set(currentRampedStrafe);
      //centerLeader.set(squaredStrafe);
    }
    // rightLeader.set(0.5);
    // leftLeader.set(0.5);
    // centerLeader.set(0.5);
    lastStrafe = currentRampedStrafe;
  }

  public float getYaw() {
    return navX.getYaw();
  }

  public void zeroYaw() {
    navX.zeroYaw();
  }

 /* public double getEncoderDistance (DriveSide driveSide) {
    switch (driveSide) {
      case LEFT:
        return leftLeader.getEncoder().getPosition() / 42 *
          RobotMap.SIDE_GEAR_RATIO * RobotMap.SIDE_WHEEL_DIAMETER;
      case RIGHT:
        return leftLeader.getEncoder().getPosition() / 42 *
          RobotMap.SIDE_GEAR_RATIO * RobotMap.SIDE_WHEEL_DIAMETER;
      case CENTER:
        return leftLeader.getEncoder().getPosition() / 42 *
          RobotMap.CENTER_GEAR_RATIO * RobotMap.CENTER_WHEEL_DIAMETER;
      default:
        return 0.0;
    }
  }*/

  public void resetTurnI () {
    turningController.setF(0.2);
    turningController.reset();
  }

  public double turnSpeedCalc(double error) {
    //if (error <= 5.0 && error >= -5.0) {return 0.0;}
   // if (error / 110.0 <= 0.4) return 0.25 * Math.signum(error);  //have 30 degrees be the cutoff point
    return 0.021 * error;
  }

  public double turnIDist;
  public double turnSpeedCalcNew(double error) {
    //if (error <= 5.0 && error >= -5.0) {return 0.0;}
   // if (error / 110.0 <= 0.4) return 0.25 * Math.signum(error);  //have 30 degrees be the cutoff point
    turnIDist += error;
    return 0.02 * error + (0.0008 * turnIDist);
  }

  public double driveSpeedCalcPlace(double error) {
    if (error <= 45) {return 0.0;}
    else return error / 75.0 * Math.signum(error); //shouldn't need signum, but just in case we do ever use (-) numbers...
  }

  public double driveSpeedCalc(double error) {
    return error / 75.0 * Math.signum(error); //shouldn't need signum, but just in case we do ever use (-) numbers...
  }

  public double strafeSpeedCalc (double error) {
    //if (error <= 5.0 && error >= -5.0) {return 0.0;}
    return 0.05 * error;
  }

  public double strafeIDist;
  public double strafeSpeedCalcNew (double error) {
    strafeIDist += error;
    return (0.03 * error) + (0.00033 * strafeIDist);
  }

  public double alignSpeedCalc (double error) {
    return 0.02 * error;
  }




  public void enableTurningControl(double angle, double tolerance) {
    double startAngle = this.getYaw();
    double temp = startAngle + angle;
    temp = temporaryFixDegrees(temp);
    turningController.setSetpoint(temp);
    turningController.enable();
    turningController.setInputRange(-180.0, 180.0);
    turningController.setOutputRange(-1.0,1.0);
    turningController.setAbsoluteTolerance(tolerance);
    turningController.setToleranceBuffer(1);
    turningController.setContinuous(true);
    turningController.setSetpoint(temp);
  }

  public double getTurnError() {
    return turningController.getError();
  }

  public void resetTurningPID() {
    turningController.setI(0.0013);
  }

  public double getTurnOutput() {
      return this.turnOutput;
    }

    public double turnError() {
      return turningController.getError();
    }
    public boolean isFieldCentric(){
      return fieldCentric;
  }

  public void setFieldCentric(boolean set){
      fieldCentric = set;
  }

  double temporaryFixDegrees(double input) {
      if (input > 180) {return input - 360;}
      else if (input < -180){return input + 360;}
      else {return input;}
  }

  public void setLEDRing (boolean on) {
    spike.set(on ? Value.kForward : Value.kReverse);
  }

  public String leftLeaderInfo() {
    return "CenterFollower: " + rightFollower.isFollower();
  }

  public void setAutoRampRate (double secondsToMaxSpeed) {
    leftLeader.setOpenLoopRampRate(secondsToMaxSpeed);
    rightLeader.setOpenLoopRampRate(secondsToMaxSpeed);
  }

  public void disableRampRate () {
    leftLeader.setOpenLoopRampRate(0);
    rightLeader.setOpenLoopRampRate(0);
  }

  @Override
  public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
  }

  
}