package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
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

    public SpinUpShooter() {
        RPM = Config.RPM.get();
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
        SmartDashboard.putNumber("Shooter RPM", shooterSubsystem.getRPM());
        SmartDashboard.putNumber("Shooter motor temp", shooterSubsystem.getTemperature());
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
