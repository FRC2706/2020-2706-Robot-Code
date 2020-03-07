package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class MoveArmToSetpointCommand extends CommandBase {

    int setPointIndex;

    public MoveArmToSetpointCommand(int setPointIndex) {
        this.setPointIndex = setPointIndex;
        addRequirements(ArmSubsystem.getInstance());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ArmSubsystem.getInstance().setpoint(setPointIndex);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return ArmSubsystem.getInstance().reachedSetpoint(setPointIndex);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
