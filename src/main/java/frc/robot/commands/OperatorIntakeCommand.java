package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConditionalSubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;


public class OperatorIntakeCommand extends CommandBase {
    
    private final static String BUTTON_PRESSED = "buttonPressed";
    
    private IntakeSubsystem intakeSubsystem;
    private ConditionalSubsystemBase.SubsystemCondition condition;
    
    public OperatorIntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(intakeSubsystem);
     //   addRequirements(ArmSubsystem.getInstance());
        
        // Initialize the condition
        condition = intakeSubsystem.getCondition("operatorActivated");
    }
    
    @Override
    public void initialize() {
        // When the command starts, tell the intake it can go
        condition.setState(true);
        ArmSubsystem.getInstance().armLowerLimitOverride = true;

    }
    
    @Override
    public void execute() {
        ArmSubsystem.getInstance().moveArm(-0.3);

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // When the command stops, tell the intake to not go
        AgitatorSubsystem.INSTANCE.stopAgitator();
        ArmSubsystem.getInstance().armLowerLimitOverride = false;
        condition.setState(false);
    }
}
