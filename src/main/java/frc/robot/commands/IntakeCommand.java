package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends CommandBase {
    
    private IntakeSubsystem intakeSubsystem;
    
    public IntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(intakeSubsystem);
        
        // Initialize the subsystem with this condition
        intakeSubsystem.setCondition("buttonPressed", false);
        intakeSubsystem.setConditionActive("buttonPressed", IntakeSubsystem.ConditionActive.ONLY_TELEOP);
    }
    
    @Override
    public void initialize() {
        // When the command starts, tell the intake it can go
        intakeSubsystem.setCondition("buttonPressed", true);
    }
    
    @Override
    public void execute() {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // When the command stops, tell the intake to not go
        intakeSubsystem.setCondition("buttonPressed", false);
    }
}
