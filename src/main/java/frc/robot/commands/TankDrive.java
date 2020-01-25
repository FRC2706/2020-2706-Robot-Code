/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;


public class TankDrive extends CommandBase {

  private final Supplier<Double> leftSpeed;
  private final Supplier<Double> rightSpeed;

  private final boolean squareInputs;
  private final boolean initBrake;

  /**
   * Creates a new TankDrive.
   */
  public TankDrive(Supplier<Double> leftSpeed, Supplier<Double> rightSpeed, boolean squareInputs, boolean initBrake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBase.getInstance());

    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    this.squareInputs = squareInputs;
    this.initBrake = initBrake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    DriveBase.getInstance().setOpenLoopVoltage();
    DriveBase.getInstance().setBrakeMode(initBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriveBase.getInstance().tankDrive(leftSpeed.get(),rightSpeed.get(),squareInputs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveBase.getInstance().setDisabledMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
