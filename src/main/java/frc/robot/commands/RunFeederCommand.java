package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;


public class RunFeederCommand extends CommandBase {

    private double speed;
    public RunFeederCommand(double speed) {
        addRequirements(FeederSubsystem.getInstance());
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        FeederSubsystem.getInstance().runFeeder(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        FeederSubsystem.getInstance().stopFeeder();
    }
}
