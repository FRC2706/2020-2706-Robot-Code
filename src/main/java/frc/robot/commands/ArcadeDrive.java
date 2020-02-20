/*----------------------------------------------------------------------------*/ 
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ 
/* Open Source Software - may be modified and shared by FRC teams. The code   */ 
/* must be accompanied by the FIRST BSD license file in the root directory of */ 
/* the project.                                                               */ 
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DriveBaseHolder;

/**
 * Drive the robot using values for driving forward and rotation (Arcade Drive)
 */
public class ArcadeDrive extends CommandBase {

  private final Supplier<Double> forwardVal;
  private final Supplier<Double> rotateVal;

  private final boolean squareInputs;
  private final boolean initBrake;

  private final DriveBase driveBase;

/**
 * Creates the arcade drive
 * 
 * @param forwardVal The values to use for driving forward
 * @param rotateVal the values to use for rotating
 * @param squareInputs Whether or not to square the forward and rotation values
 * @param initBrake whether or not to start and end the command in brake or coast mode
 */
  public ArcadeDrive(Supplier<Double> forwardVal, Supplier<Double> rotateVal, boolean squareInputs, boolean initBrake) {
    // Ensure that this command is the only one to run on the drive base
    // Requires must be included to use this command as a default command for the drive base
    this.forwardVal = forwardVal;
    this.rotateVal = rotateVal;
    this.squareInputs = squareInputs;
    this.initBrake = initBrake;
    this.driveBase = DriveBaseHolder.getInstance();
    addRequirements(this.driveBase);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Prepare for driving by human
    this.driveBase.setDriveMode(DriveBase.DriveMode.OpenLoopVoltage);
    this.driveBase.setNeutralMode(initBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pass values to drive base to make the robot move
    this.driveBase.arcadeDrive(forwardVal.get(), rotateVal.get(), squareInputs);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Go back to disabled mode
    this.driveBase.setDriveMode(DriveBase.DriveMode.Disabled);
  }
}
