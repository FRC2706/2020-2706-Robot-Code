package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

import java.util.HashMap;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this IntakeSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();
    
    // The supplier of the intake speed
    private final static FluidConstant<Double> INTAKE_SPEED = new FluidConstant<>("intake-target-speed", 0.25d)
            .registerToTable(Config.constantsTable);
    
    // A dictionary to hold all the conditions for the intake to run
    private HashMap<String, Boolean> conditions;
    private HashMap<String, Condition> conditionsActiveDuring;
    
    // The intake motor (if any)
    private VictorSPX intakeMotor;
    
    public enum Condition {
        ONLY_AUTO,
        ONLY_TELEOP,
        BOTH;
    }
    
    /**
     * Creates a new instance of this IntakeSubsystem.
     * This constructor is private since this class is a Singleton. External classes 
     * should use the {@link #getInstance()} method to get the instance.
     */
    private IntakeSubsystem() {
        // Initialize the private variables
        conditions = new HashMap<>();
        conditionsActiveDuring = new HashMap<>();
        if (Config.INTAKE_MOTOR != -1) {
            intakeMotor = new VictorSPX(Config.INTAKE_MOTOR);
        }
    }
    
    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method 
     * should be used -- {@code IntakeSubsystem.getInstance();} -- by external
     * classes, rather than the constructor to get the instance of this class.
     */
    public static IntakeSubsystem getInstance() {
        return INSTANCE;
    }
    
    /**
     * Sets the condition of the name to the provided value. The intake will only run if all conditions are met
     * @param name The condition name. e.g. 'Button'
     * @param condition The condition value.
     */
    public void setCondition(String name, boolean condition) {
        conditions.put(name, condition);
    }
    
    /**
     * Sets whether the condition should be disregarded during auto or teleop
     * @param name The condition name
     * @param during When should it be active
     */
    public void setConditionActive(String name, Condition during) {
        conditionsActiveDuring.put(name, during);
    }
    
    @Override
    public void periodic() {
        // The intakeMotor will be null if the Config entry for it was -1. (Meaning this robot doesn't have an intake)
        if (intakeMotor == null) return;
        
        boolean isAuto = DriverStation.getInstance().isAutonomous();
        
        // Check if all conditions are met: we can run if none of the conditions match the following filter
        boolean canRun = conditions.entrySet().stream().noneMatch(c -> {
            // If we're in auto ignore the teleop conditions and vice versa
            Condition ignored = (isAuto ? Condition.ONLY_TELEOP : Condition.ONLY_AUTO);
            
            // If we're not ignoring the condition and it is false return true
            // If the condition's active during time isn't set, assume it's BOTH
            if (!conditionsActiveDuring.containsKey(c.getKey())) {
                return !c.getValue();
            } else {
                return conditionsActiveDuring.get(c.getKey()) != ignored && !c.getValue();
            }
        });
        
        // If all the conditions are met, set the motor to run at the target speed, otherwise stop.
        if (canRun) {
            intakeMotor.set(ControlMode.PercentOutput, INTAKE_SPEED.get());
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0d);
        }
    }
}

//hi
