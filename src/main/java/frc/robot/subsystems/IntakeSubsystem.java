package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

import java.util.HashMap;
import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this IntakeSubsystem. External classes should
     * use the {@link #getInstance()} method to get the instance.
     */
    private final static IntakeSubsystem INSTANCE = new IntakeSubsystem();
    
    // The supplier of the intake speed
    private Supplier<Double> intakeSpeed;
    
    // A dictionary to hold all the conditions for the intake to run
    private HashMap<String, Boolean> conditions;
    
    // The intake motor (if any)
    private VictorSPX intakeMotor;
    
    /**
     * Creates a new instance of this IntakeSubsystem.
     * This constructor is private since this class is a Singleton. External classes 
     * should use the {@link #getInstance()} method to get the instance.
     */
    private IntakeSubsystem() {
        // Initialize the private variables
        conditions = new HashMap<>();
        intakeSpeed = Config.INTAKE_SPEED;
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
    
    @Override
    public void periodic() {
        // The intakeMotor will be null if the Config entry for it was -1. (Meaning this robot doesn't have an intake)
        if (intakeMotor == null) return;
        
        // Check if all conditions are met.
        // e.g. The intake button is held down AND there is space for more balls
        boolean canRun = true;
        for (var condition : conditions.values()) {
            if (!condition) {
                canRun = false;
                break;
            }
        }
        
        // If all the conditions are met, set the motor to run at the target speed, otherwise stop.
        if (canRun) {
            intakeMotor.set(ControlMode.PercentOutput, intakeSpeed.get());
        } else {
            intakeMotor.set(ControlMode.PercentOutput, 0d);
        }
    }
}

