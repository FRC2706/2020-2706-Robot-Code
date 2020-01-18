/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class ArcadeDrive extends CommandBase {

  private final Supplier<Double> forwardVal;
  private final Supplier<Double> rotateVal;

  private final boolean squareInputs;
  

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Supplier<Double> forwardVal, Supplier<Double> rotateVal, boolean squareInputs) {

    addRequirements(DriveBase.getInstance());
    this.forwardVal = forwardVal;
    this.rotateVal = rotateVal;
    this.squareInputs = squareInputs;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriveBase.getInstance().arcadeDrive(forwardVal.get(), rotateVal.get(), squareInputs);
    

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
