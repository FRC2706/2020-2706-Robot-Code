/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;

public class ArcadeDriveWithJoystick extends ArcadeDrive {
  /**
   * Creates a new ArcadeDriveWithJoystick.
   */
  public ArcadeDriveWithJoystick(Joystick joy1, int axis1, boolean invert1, Joystick joy2, int axis2, boolean invert2) {
    super(() -> sign(Config.removeJoystickDeadband(joy1.getRawAxis(axis1)), invert1), () -> sign(Config.removeJoystickDeadband(joy1.getRawAxis(axis2)), invert2), false, true);
  }

  /**
   * Creates a new ArcadeDriveWithJoystick with only one Joystick
   */
  public ArcadeDriveWithJoystick(Joystick joy, int axis1, boolean invert1, int axis2, boolean invert2){
    this(joy, axis1, invert1, joy, axis2, invert2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { return false; }

  /**
   * Inverts a given double
   *
   * @param number The original number
   * @param sign Weather or not to invert it
   * @return The inverted value
   */
  private static double sign(double number, boolean sign){
    if(sign){
      return -number;
    }
    else{
      return number;
    }

  }
}
