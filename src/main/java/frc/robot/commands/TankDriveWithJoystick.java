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

public class TankDriveWithJoystick extends TankDrive {
  /**
   * Creates a new TankDriveWithJoystick.
   */
  public TankDriveWithJoystick(Joystick joy1, int axis1, boolean invert1, Joystick joy2, int axis2, boolean invert2) {
    // Use addRequirements() here to declare subsystem dependencies.

    super(() -> sign(joy1.getRawAxis(axis1), invert1), () -> sign(joy2.getRawAxis(axis2), invert2), Config.TELEOP_SQUARE_JOYSTICK_INPUTS, Config.TELEOP_BRAKE);

  }

  public TankDriveWithJoystick(Joystick joy, int axis1, boolean invert1, int axis2, boolean invert2) {

    this(joy, axis1, invert1, joy, axis2, invert2);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
  private static double sign(double number, boolean sign) {

    if (sign) {

        return -number;

    } else {

        return number;

    }

}
}
