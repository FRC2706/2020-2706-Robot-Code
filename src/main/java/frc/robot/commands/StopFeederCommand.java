package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;


public class StopFeederCommand extends CommandBase {

    public StopFeederCommand() {
        addRequirements(FeederSubsystem.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        FeederSubsystem.getInstance().stopFeeder();

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
