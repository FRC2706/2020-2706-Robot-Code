package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooterWithTime extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    private boolean doneRamping;

    private int RPM;
    private int time;

    private Timer timer = new Timer();

    public SpinUpShooterWithTime(int RPM, int time) {
        shooterSubsystem = ShooterSubsystem.getInstance();
        this.RPM = RPM;
        this.time = time;
        if (shooterSubsystem.isActive()) {
            addRequirements(shooterSubsystem);
        }
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(RPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTargetRPM(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }
}
