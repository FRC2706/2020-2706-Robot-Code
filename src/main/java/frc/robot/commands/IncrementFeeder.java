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

    /**
     * Creates a new FeederSubsystem.
     */
    public IncrementFeeder() {
        System.out.println("Created increment feeder command");
        addRequirements(FeederSubsystem.getInstance());
        this.feeder = FeederSubsystem.getInstance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        FeederSubsystem.zeroTalon();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Executing incrementing command!");
     //   feeder.runFeeder();
     //   this.feeder.incrementPowerCells();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending command..");
        this.feeder.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return feeder.doneIncrementing();
    }

}