/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj.Timer;



public class DriveWithTime extends CommandBase {

  public double seconds = 0;
  public double leftSpeeds= 0;
  public double rightSpeeds = 0;

  /**
   * Creates a new DriveWithTime.
   */

  public DriveWithTime(double seconds, double leftSpeed, double rightSpeed) {

    this.seconds = seconds;
    this.leftSpeeds = leftSpeed;
    this.rightSpeeds = rightSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    System.out.println("DriveWithTime Running");
  }
  private Timer m_timer = new Timer();
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    
    addRequirements(DriveBase.getInstance());

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    DriveBase.getInstance().tankDrive(leftSpeeds,rightSpeeds,false);
    
  }
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > seconds;
  }
}
