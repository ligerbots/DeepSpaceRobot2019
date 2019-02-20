/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.ligerbots.robot.Robot;
import org.ligerbots.robot.RobotMap;


@SuppressWarnings("deprecation")
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  enum DriveSide {
    LEFT, RIGHT, CENTER
  }

  WPI_TalonSRX leftLeaderTalon;
  WPI_TalonSRX leftFollowerTalon;
  WPI_TalonSRX rightLeaderTalon;
  WPI_TalonSRX rightFollowerTalon;
  WPI_TalonSRX centerLeaderTalon;
  WPI_TalonSRX centerFollowerTalon;
  CANSparkMax leftLeaderSpark;
  CANSparkMax leftFollowerSpark;
  CANSparkMax rightLeaderSpark;
  CANSparkMax rightFollowerSpark;
  CANSparkMax centerLeaderSpark;
  CANSparkMax centerFollowerSpark;
  DifferentialDrive diffDrive;
  Boolean fieldCentric = false;
  PIDController turningController;
  AHRS navX;

  double turnOutput;

  Relay spike;


  public DriveTrain () {
    if (!Robot.isSecondRobot) {
      leftLeaderSpark = new CANSparkMax(14, MotorType.kBrushless);
      leftFollowerSpark = new CANSparkMax(1, MotorType.kBrushless);
      rightLeaderSpark = new CANSparkMax(2, MotorType.kBrushless);
      rightFollowerSpark = new CANSparkMax(3, MotorType.kBrushless);
      centerLeaderSpark = new CANSparkMax(4, MotorType.kBrushless);
      centerFollowerSpark = new CANSparkMax(5, MotorType.kBrushless);
    }
    else {
      leftLeaderTalon = new WPI_TalonSRX(14);
      leftFollowerTalon = new WPI_TalonSRX(1);
      rightLeaderTalon = new WPI_TalonSRX(2);
      rightFollowerTalon = new WPI_TalonSRX(3);
      centerLeaderTalon = new WPI_TalonSRX(4);
      centerFollowerTalon = new WPI_TalonSRX(5);
    }
    if (!Robot.isSecondRobot) {
      leftLeaderSpark.setInverted(false);
      leftFollowerSpark.follow(leftLeaderSpark, false);
      rightFollowerSpark.follow(rightLeaderSpark);
      centerFollowerSpark.follow(centerLeaderSpark);
    }
    else {
      leftLeaderTalon.setInverted(false);
      leftFollowerTalon.set(ControlMode.Follower, 14);
      rightFollowerTalon.set(ControlMode.Follower, 2);
      centerFollowerTalon.set(ControlMode.Follower, 4); //MIGHT NEED TO BE INVERTED
    }
    if (!Robot.isSecondRobot) {
      diffDrive = new DifferentialDrive(leftLeaderSpark, rightLeaderSpark); //fix l8r
    }
    else {
      diffDrive = new DifferentialDrive(leftLeaderSpark, rightLeaderTalon); //fix l8r
    }
    navX = new AHRS(Port.kMXP, (byte) 200);

    spike = new Relay(1);

    if (!Robot.isSecondRobot) {
      centerLeaderSpark.setOpenLoopRampRate(0.005);
      centerLeaderSpark.setSmartCurrentLimit(20);
      Arrays.asList(leftLeaderSpark, leftFollowerSpark, rightLeaderSpark, rightFollowerSpark, centerLeaderSpark, centerFollowerSpark)
         .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(25));
    }
 //
    else {
      centerLeaderTalon.configPeakCurrentLimit(20);
      Arrays.asList(leftLeaderTalon, leftFollowerTalon, rightLeaderTalon, rightFollowerTalon, centerLeaderTalon, centerFollowerTalon)
      .forEach((WPI_TalonSRX talon) -> talon.configPeakCurrentLimit(25));
    }
    //STILL NEED TO SET CURRENT LIMITS!!!!
    
   // turningController = new PIDController(0.045, 0.004, 0.06, navX, output -> this.turnOutput = output);

  }
  
  double squaredStrafe;
  public void allDrive (double throttle, double rotate, double strafe) {
    squaredStrafe = strafe * strafe * Math.signum(strafe);
    if (fieldCentric) {
      diffDrive.arcadeDrive(throttle * Math.cos(getYaw() + squaredStrafe * Math.sin(getYaw())), rotate);
      if (!Robot.isSecondRobot) {
        centerLeaderSpark.set(-throttle * Math.sin(getYaw()) + squaredStrafe * Math.cos(getYaw()));
      }
      else {
        centerLeaderTalon.set(-throttle * Math.sin(getYaw()) + squaredStrafe * Math.cos(getYaw()));
      }
    }
    else {
      diffDrive.arcadeDrive(throttle, -rotate);
      if (!Robot.isSecondRobot) {
        centerLeaderSpark.set(squaredStrafe / 2.2);
      }
      else {
        centerLeaderTalon.set(squaredStrafe / 2.2);
      }
      
    }
   // rightLeader.set(0.5);
   // leftLeader.set(0.5);
   // centerLeader.set(0.5);
  }

  public float getYaw() {
    return navX.getYaw();
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

  public double turnSpeedCalc(double error) {
    //if (error <= 5.0 && error >= -5.0) {return 0.0;}
    if (error / 90.0 <= 0.4) return 0.4 * Math.signum(error);  //have 30 degrees be the cutoff point
    return error / 90.0 * Math.signum(error);
  }

  public double driveSpeedCalc(double error) {
    if (error <= 40) {return 0.0;}
    else if (error / 85.0 <= 0.4) return 0.4 * Math.signum(error);  //have 24 inches be the cutoff point
    return error / 85.0 * Math.signum(error); //shouldn't need signum, but just in case we do ever use (-) numbers...
  }

  public double strafeSpeedCalc (double error) {
    //if (error <= 5.0 && error >= -5.0) {return 0.0;}
    if (error > 25) {return 0.9 * Math.signum(error);}
    else if (error > 15) {return 0.45 * Math.signum(error);}
    return 0.4 * Math.signum(error);
  }

  public void enableTurningControl(double angle, double tolerance) {
    double startAngle = this.getYaw();
    double temp = startAngle + angle;
   // RobotMap.TURN_P = turningController.getP();
   // RobotMap.TURN_D = turningController.getD();
   // RobotMap.TURN_I = turningController.getI();
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

  public double getTurnOutput() {
    return this.turnOutput;
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
  return "CenterFollower: " + rightFollowerSpark.getOutputCurrent();//I guess this is what it should be?
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
