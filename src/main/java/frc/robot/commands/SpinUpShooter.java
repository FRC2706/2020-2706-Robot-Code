package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    private boolean doneRamping;

    private int RPM;

    public SpinUpShooter(int RPM) {
        shooterSubsystem = ShooterSubsystem.getInstance();
        this.RPM = RPM;
        if (shooterSubsystem.isActive()) {
            addRequirements(shooterSubsystem);
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(RPM);
        doneRamping = shooterSubsystem.isAtTargetRPM();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTargetRPM(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !shooterSubsystem.isActive();
    }
}
