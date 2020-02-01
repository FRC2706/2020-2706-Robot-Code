package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SubsystemOperation;


public class OperatorIntakeCommand extends CommandBase {
    
    private final static String BUTTON_PRESSED = "buttonPressed";
    
    private IntakeSubsystem intakeSubsystem;
    private SubsystemOperation condition;
    
    public OperatorIntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(intakeSubsystem);
        
        // Initialize the condition
        condition = intakeSubsystem.getConditionManager()
                // This operation only applies while the robot is under Operator Control (TELEOP)
                .createOperation(BUTTON_PRESSED, SubsystemOperation.State.TELEOP, false);
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
