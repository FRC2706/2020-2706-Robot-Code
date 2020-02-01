/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
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

    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        DriveBase.init();

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

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {

    }
    else {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);

    //send start up signal to vision team
    visionControlNetTable.startUpVision();

    bFromTeleMode = false;
    bRealMatch = isRealMatch();
  }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
    
    //set teleop mode to true
    bFromTeleMode = true;

    else {
    joystick.setRumble(RumbleType.kLeftRumble, 0);
    joystick.setRumble(RumbleType.kRightRumble, 0);
  }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

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

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}