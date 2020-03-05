package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;


public class MoveToShooter extends CommandBase {

    private int ticks;

    public MoveToShooter(int ticks) {
        this.ticks = ticks;
    }

    @Override
    public void initialize() {
        //System.out.println("Init");
        addRequirements(FeederSubsystem.getInstance());
        FeederSubsystem.zeroTalon();
    }

    @Override
    public void execute() {
        FeederSubsystem.getInstance().incrementPowerCells(ticks);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return FeederSubsystem.getInstance().doneIncrementing(ticks);
    }

    @Override
    public void end(boolean interrupted) {
        FeederSubsystem.getInstance().stopFeeder();
    }
}
