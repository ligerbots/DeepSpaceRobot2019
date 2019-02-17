/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  AHRS navX;

  double turnOutput;


  public DriveTrain () {

    leftLeader = new CANSparkMax(14, MotorType.kBrushless);
    leftFollower = new CANSparkMax(1, MotorType.kBrushless);
    rightLeader = new CANSparkMax(2, MotorType.kBrushless);
    rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    centerLeader = new CANSparkMax(4, MotorType.kBrushless);
    centerFollower = new CANSparkMax(5, MotorType.kBrushless);

    leftLeader.setInverted(true);
    leftFollower.follow(leftLeader, true);
    rightFollower.follow(rightLeader);
    centerFollower.follow(centerLeader); //MIGHT NEED TO BE INVERTED

    diffDrive = new DifferentialDrive(leftLeader, rightLeader); //fix l8r

    navX = new AHRS(Port.kMXP, (byte) 200);

   // turningController = new PIDController(0.045, 0.004, 0.06, navX, output -> this.turnOutput = output);

  }

  public void allDrive (double throttle, double rotate, double strafe) {
   /* if (fieldCentric) {
      diffDrive.arcadeDrive(throttle * Math.cos(getYaw() + strafe * Math.sin(getYaw())), rotate);
      centerLeader.set(-throttle * Math.sin(getYaw()) + strafe * Math.cos(getYaw()));
    }
    else {*/
      diffDrive.arcadeDrive(throttle, -rotate);
      centerLeader.set(strafe);
   // }
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

  double temporaryFixDegrees(double input) {
    if (input > 180) {return input - 360;}
    else if (input < -180){return input + 360;}
    else {return input;}
}

public String leftLeaderInfo() {
  return "CenterFollower: " + rightFollower.isFollower();
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
