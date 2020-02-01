package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SubsystemCondition;


public class OperatorIntakeCommand extends CommandBase {
    
    private IntakeSubsystem intakeSubsystem;
    private SubsystemCondition condition;
    
    public OperatorIntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(intakeSubsystem);
        
        // Initialize the condition
        condition = intakeSubsystem.getConditionManager()
                .createCondition("buttonPressed", SubsystemCondition.ConditionOptions.TELEOP, false);
    }
    
    @Override
    public void initialize() {
        // When the command starts, tell the intake it can go
        condition.setEnabled(true);
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
        condition.setEnabled(false);
    }
}
