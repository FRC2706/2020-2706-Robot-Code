
package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.config.Config;

/**
 * Drives the robot using joystick axes for driving forward and rotation
 */
public class ArcadeDriveWithJoystick extends ArcadeDrive {
  /**
   * Drives using axes that are on separate joysticks
   * 
   * @param joy1 The jostick that the first axis is on
   * @param axis1 The first axis
   * @param invert1 Whether or not to negate the first joystick input
   * @param joy2  The joystick that the second ais is on
   * @param axis2 The second axis
   * @param invert2 Whether to negate the second joystick input
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
