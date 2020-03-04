/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class IncrementFeeder extends CommandBase {

    FeederSubsystem feeder;
    private Double incrementTicks;

    /**
     * Creates a new FeederSubsystem.
     */
    public IncrementFeeder(Double incrementTicks) {
        addRequirements(FeederSubsystem.getInstance());
        this.feeder = FeederSubsystem.getInstance();
        this.incrementTicks = incrementTicks;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        FeederSubsystem.zeroTalon();
    }

    @Override
    public void execute() {
        this.feeder.runFeeder();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.feeder.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return feeder.doneIncrementing(incrementTicks);
    }

}