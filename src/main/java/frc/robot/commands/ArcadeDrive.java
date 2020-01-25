/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveBase;

public class ArcadeDrive extends CommandBase {

  private final Supplier<Double> forwardVal;
  private final Supplier<Double> rotateVal;

  private final boolean squareInputs;
  private final boolean initBrake;
  

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Supplier<Double> forwardVal, Supplier<Double> rotateVal, boolean squareInputs, boolean initBrake) {

    addRequirements(DriveBase.getInstance());
    this.forwardVal = forwardVal;
    this.rotateVal = rotateVal;
    this.squareInputs = squareInputs;
    this.initBrake = initBrake;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Prepare for driving by human
    DriveBase.getInstance().setOpenLoopVoltage();
    DriveBase.getInstance().setBrakeMode(initBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveBase.getInstance().arcadeDrive(forwardVal.get(), rotateVal.get(), squareInputs);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveBase.getInstance().setDisabledMode();
  }

}
