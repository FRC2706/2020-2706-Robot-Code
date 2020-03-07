package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RunFeederCommandWithTime extends CommandBase {

    private double speed;
    private double time;

    Timer timer = new Timer();
    public RunFeederCommandWithTime(double speed, double time) {
        addRequirements(FeederSubsystem.getInstance());
      //  addRequirements(ShooterSubsystem.getInstance());
        this.speed = speed;
        this.time = time;
    }

    @Override
    public void initialize() {
        timer.start();

    }

    @Override
    public void execute() {
        if(ShooterSubsystem.getInstance().isAtTargetRPM()) {
            FeederSubsystem.getInstance().runFeeder(speed);
        } else {
            FeederSubsystem.getInstance().stopFeeder();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return timer.get() > time;
    }

    @Override
    public void end(boolean interrupted) {
        FeederSubsystem.getInstance().stopFeeder();
    }
}
