<<<<<<< HEAD
package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.config.Config;
/**
 * Allows the robot to be driven using curve drive
 */
public class CurvatureDriveWithJoystick extends CurvatureDrive {

    /**
     * Drives using axes that are on separate joysticks
     *
     * @param joy1    The joystick that the first axis is on
     * @param axis1   The first axis
     * @param invert1 Whether to negate the first joystick input
     * @param joy2    The joystick that the second axis is on
     * @param axis2   The second axis
     * @param invert2 Whether to negate the second joystick input
     */
    public CurvatureDriveWithJoystick(Joystick joy1, int axis1, boolean invert1,
                                      Joystick joy2, int axis2, boolean invert2,
                                      Joystick joy3, int button) {
        super(() -> sign(joy1.getRawAxis(axis1), invert1), () -> sign(joy2.getRawAxis(axis2), invert2),
                Config.TELEOP_BRAKE, () -> joy3.getRawButton(button), Config.TELEOP_SQUARE_JOYSTICK_INPUTS);
    }

    /**
     * Drives using axes that are on separate joysticks
     *
     * @param joy1         The joystick that the first axis is on
     * @param axis1        The first axis
     * @param invert1      Whether to negate the first joystick input
     * @param joy2         The joystick that the second axis is on
     * @param axis2        The second axis
     * @param invert2      Whether to negate the second joystick input
     * @param squareInputs Whether inputs should be squared
     */
    public CurvatureDriveWithJoystick(Joystick joy1, int axis1, boolean invert1,
                                      Joystick joy2, int axis2, boolean invert2,
                                      Joystick joy3, int button, boolean squareInputs) {
        super(() -> sign(joy1.getRawAxis(axis1), invert1), () -> sign(joy2.getRawAxis(axis2), invert2),
                Config.TELEOP_BRAKE, () -> joy3.getRawButton(button), squareInputs);
    }

    /**
     * Drives using axes that are on the same joystick
     *
     * @param joy     The joystick with the axes
     * @param axis1   The first axis
     * @param invert1 Whether to negate the first joystick input
     * @param axis2   The second axis
     * @param invert2 Whether to negate the second joystick input
     */
    public CurvatureDriveWithJoystick(Joystick joy, int axis1, boolean invert1, int axis2, boolean invert2, int button) {
        this(joy, axis1, invert1, joy, axis2, invert2, joy, button);
    }

    @Override
    public boolean isFinished() {
        // The command should only finish when cancelled from somewhere else
        return false;
    }

    /**
     * Conditionally negates a number with a boolean
     *
     * @param number The number to negate
     * @param sign   True to negate the number
     * @return The negated number
     */
    private static double sign(double number, boolean sign) {
        if (sign) {
            return -number;
        } else {
            return number;
        }
    }
}
=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CurvatureDriveWithJoystick extends CommandBase {
  /**
   * Creates a new CurvatureDriveWithJoystick.
   */
  public CurvatureDriveWithJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
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
}
>>>>>>> AlexDifferentialDriveTest
