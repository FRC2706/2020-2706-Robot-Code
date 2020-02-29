package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends CommandBase {

    private ShooterSubsystem shooterSubsystem;

    boolean doneRamping;
    
    int RPM = 2000;

    public SpinUpShooter() {
        shooterSubsystem = ShooterSubsystem.getInstance();
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
        if (doneRamping) {
            // Print to console
            System.out.println("calculatedRPM is within 30 units of targetRPM");
        }
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
