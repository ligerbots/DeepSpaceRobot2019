/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot2019.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.ligerbots.robot2019.FieldPosition;

/**
 * Add your docs here.
 */
@SuppressWarnings("deprecation")
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  enum DriveSide {
    LEFT, RIGHT
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

    leftLeader = new CANSparkMax(0, MotorType.kBrushless);
    leftFollower = new CANSparkMax(1, MotorType.kBrushless);
    rightLeader = new CANSparkMax(2, MotorType.kBrushless);
    rightFollower = new CANSparkMax(3, MotorType.kBrushless);
    centerLeader = new CANSparkMax(4, MotorType.kBrushless);
    centerFollower = new CANSparkMax(5, MotorType.kBrushless);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
    centerFollower.follow(centerLeader); //MIGHT NEED TO BE INVERTED

    diffDrive = new DifferentialDrive(leftLeader, rightLeader);

    navX = new AHRS(Port.kMXP, (byte) 200);

    turningController = new PIDController(0.045, 0.004, 0.06, navX, output -> this.turnOutput = output);

  }

  public void allDrive (double throttle, double rotate, double strafe) {
    if (fieldCentric) {
      diffDrive.arcadeDrive(throttle * Math.cos(getYaw() + strafe * Math.sin(getYaw())), rotate);
      centerLeader.set(-throttle * Math.sin(getYaw()) + strafe * Math.cos(getYaw()));
    }
    else {
      diffDrive.arcadeDrive(throttle, rotate);
      centerLeader.set(strafe);
    }
  }

  public float getYaw() {
    return navX.getYaw();
  }

  public double getEncoderDistance (DriveSide driveSide) {
    return 0; //TODO
  }

  public double turnSpeedCalc (double error) {
    if (error > 30) {return 1;}
    else if (error > 10) {return 0.3;}
    return 0.2;
  }

  public double driveSpeedCalc (double error) {
    if (error > 24) {return 1;}
    else if (error > 12) {return 0.7;}
    return 0.45;
  }

  public FieldPosition getRobotPosition () {
    return new FieldPosition(0,0); //TODO
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
