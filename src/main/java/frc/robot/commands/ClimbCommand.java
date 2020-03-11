/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends CommandBase {

    private ClimberSubsystem climberSubsystem;

    private double climbSpeed;

    public ClimbCommand(double climbSpeed) {
        climberSubsystem = ClimberSubsystem.getInstance();
        this.climbSpeed = climbSpeed;

        if (climberSubsystem.isActive()){
            addRequirements(climberSubsystem);
        }
    }

    @Override
    public void initialize() {
        // Perform checks? Arm in position? Arm PID?
    }

    @Override
    public void execute() {
        climberSubsystem.runClimber(climbSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimber();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
