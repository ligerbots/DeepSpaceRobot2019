/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.ligerbots.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ligerbots.robot.Commands.DriveCommand;
import org.ligerbots.robot.Commands.IntakeRunCommand;
import org.ligerbots.robot.Commands.SetIntakeCommand;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.ligerbots.robot.Subsystems.DriveTrain;
import org.ligerbots.robot.Subsystems.Elevator;
import org.ligerbots.robot.Subsystems.Grabber;
import org.ligerbots.robot.Subsystems.Intake;
import org.ligerbots.robot.Subsystems.Pneumatics;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static DriveTrain driveTrain;
  public static OI oi;
  public static Elevator elevator;
  public static Intake intake;
  public static Grabber grabber;
  public static DriveCommand driveCommand;
  public static Pneumatics compressor;
  //static SetIntakeCommand initialSetIntake;
  public static Boolean isSecondRobot;
  public static IntakeRunCommand intakeCommand;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    driveTrain = new DriveTrain();
    elevator = new Elevator();
    driveCommand = new DriveCommand();
    SmartDashboard.putString("RobotDidInit?", "Yes");
    compressor = new Pneumatics();
    intake = new Intake();
    grabber = new Grabber();
    oi = new OI();
    intakeCommand = new IntakeRunCommand();
    elevator.resetEncoderZero();

   // initialSetIntake = new SetIntakeCommand(true);
   // initialSetIntake.start();
    // Detect the motor controllers we're using
    isSecondRobot = (new TalonSRX(RobotMap.DETERMINE_WHICH_ROBOT).getFirmwareVersion() != -1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("wrist error", elevator.wrist.getClosedLoopError());
    SmartDashboard.putNumber("wrist speed", elevator.wrist.get());
    SmartDashboard.putNumber("pid error", elevator.pidController.getError());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    Scheduler.getInstance().run();
  }


  @Override
  public void teleopInit() {
    driveCommand.start();
    intakeCommand.start();
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //driveTrain.allDrive(0.5, 0.5, 0.5);
    SmartDashboard.putNumber("encoder value", elevator.encoder.getValue());
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Pressure", compressor.getPressure());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
    //Scheduler.getInstance().run();
  }
}
