/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;
import frc.robot.nettables.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Joystick joystick = new Joystick(0);
  //private double kP = 0.5;  // This was for a pid that is currently not finished...
  //private double kI = 0.5;
  //private double kD = 0.5;
  //private int setpoint = 0;
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //network table for vision control
  private VisionCtrlNetTable visionControlNetTable;
  //network table for control systems control
  private ControlCtrlNetTable controlCtrlNetTable;
  //flag to indicate from the teleop mode
  private Boolean bFromTeleMode;
  //flag to indicate the real match
  private Boolean bRealMatch;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
  CameraServer.getInstance().startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    DriveBase.init();

    m_robotContainer = new RobotContainer();
    
    //create a vision control table
    visionControlNetTable  = new VisionCtrlNetTable();
    //create a control system control table
    controlCtrlNetTable = new ControlCtrlNetTable();
    //set to false
    bFromTeleMode = false;
    bRealMatch = false;
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  public void disabledInit() {
    if (bRealMatch == true && bFromTeleMode == true) 
    {
      // if in a real match and from teleop mode
      // Write to the network table the shut down signal.
      visionControlNetTable.shutDownVision();
      controlCtrlNetTable.shutDownControl();

      System.out.println("Driver Station Disabled");      
    }

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

   //PIDController pid = new PIDController(kP, kI, kD); // This was for a pid that is currently not finished...
   //pid.setTolerance(5, 10);
   //pid.atSetpoint();
   //pid.setIntegratorRange(-0.5, 0.5);
   //pid.enableContinuousInput(-180, 180);
   //SmartDashboard.putNumber("PID Graph Test", (pid.calculate(encoder.getDistance(), setpoint)));

   boolean buttonValueA; // A is 1
   boolean buttonValueB; // B is 2
   boolean buttonValueX; // X is 3
   boolean buttonValueY; // Y is 4
   
   SmartDashboard.putNumber("Joystick X Value", joystick.getX());
   SmartDashboard.putNumber("Joystick Y Value", joystick.getY()* -1);
   SmartDashboard.putBoolean("X Too Slow?", joystick.getX() < -0.9);
   SmartDashboard.putBoolean("X Too Fast?", joystick.getX() > 0.9);
   SmartDashboard.putBoolean("Y Too Fast?", joystick.getY() < -0.9);
   SmartDashboard.putBoolean("Y Too Slow?", joystick.getY() > 0.9);

  if (joystick.getX() < -0.9) {
    joystick.setRumble(RumbleType.kLeftRumble, 1);
    joystick.setRumble(RumbleType.kRightRumble, 1);
    }
    else {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);

    //send start up signal to vision team
    visionControlNetTable.startUpVision();

    bFromTeleMode = false;
    bRealMatch = isRealMatch();
  }

  if (joystick.getX() > 0.9) {
    joystick.setRumble(RumbleType.kLeftRumble, 1);
    joystick.setRumble(RumbleType.kRightRumble, 1);
    }
    else {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);
  }

  if (joystick.getY() < -0.9) {
    joystick.setRumble(RumbleType.kLeftRumble, 1);
    joystick.setRumble(RumbleType.kRightRumble, 1);
    }
    
    //set teleop mode to true
    bFromTeleMode = true;

    else {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);
  }

  if (joystick.getY() > 0.9) {
    joystick.setRumble(RumbleType.kLeftRumble, 1);
    joystick.setRumble(RumbleType.kRightRumble, 1);
    }
    else {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    //set teleop mode to false
    bFromTeleMode = false;
  buttonValueA = joystick.getRawButton(1); // Button A
  if(buttonValueA) {SmartDashboard.putBoolean("A", buttonValueA);
  }
  buttonValueB = joystick.getRawButton(2); // Button B
  if (buttonValueB) {SmartDashboard.putBoolean("B", buttonValueB);
  }
  buttonValueX = joystick.getRawButton(3); // Button X
  if (buttonValueX) {SmartDashboard.putBoolean("X", buttonValueX);
  } 
  buttonValueY = joystick.getRawButton(4); // Button Y
  if (buttonValueY) {SmartDashboard.putBoolean("Y", buttonValueY);
  }
  
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
     * Determines if the robot is in a real match.
     *
     * @return True if the robot is in a real match, false otherwise.
     */
  public static boolean isRealMatch() {
    return DriverStation.getInstance().isFMSAttached();
  }
}

