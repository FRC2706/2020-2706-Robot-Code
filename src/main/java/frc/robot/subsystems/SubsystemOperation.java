package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class SubsystemOperation {
    // A function that represents when the operation is applicable to the subsystem
    private Supplier<Boolean> applicable;
    
    /**
     * A static class with some pre-defined applicable states for ease-of-use
     */
    public static class State {
        public static final Supplier<Boolean> TELEOP = () -> DriverStation.getInstance().isOperatorControl();
        public static final Supplier<Boolean> AUTO = () -> DriverStation.getInstance().isAutonomous();
        public static final Supplier<Boolean> TELEOP_OR_AUTO = () -> TELEOP.get() || AUTO.get();
        public static final Supplier<Boolean> ALWAYS = () -> true;
    }
    
    // The name given to this operation
    private String name;
    
    // The state of this operation
    private boolean enabled;
    
    /**
     * Constructor for a operation. DO NOT USE THIS TO MAKE A operation YOURSELF.
     * Use the {@code SubsystemOperationManager.createOperation} method instead!
     * @param name The name of the operation
     * @param applicable The options for this operation
     * @param startEnabled The start state of this operation
     */
    public SubsystemOperation(String name, Supplier<Boolean> applicable, boolean startEnabled) {
        this.name = name;
        this.applicable = applicable;
        this.enabled = startEnabled;
    }
    
    /**
     * Setter for if this operation is enabled
     * @param enabled enabled?
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    
    /**
     * @return is this operation is enabled?
     */
    public boolean getEnabled() {
        return this.enabled;
    }
    
    /**
     * @return This operation's name
     */
    public String getName() {
        return this.name;
    }
    
    /**
     * Checks the operation's applicable state
     * @return True if the operation is applicable
     */
    public boolean isApplicable() {
        return applicable.get();
    }
}
