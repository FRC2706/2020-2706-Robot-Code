package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

public abstract class ConditionalSubsystemBase extends SubsystemBase {
    
    
    /**
     * A class that represents a subsystem's condition.
     */
    public class SubsystemCondition {
        private String name;
        private boolean state;
        private Supplier<Boolean> activeWhen;
        
        protected SubsystemCondition(String name, Supplier<Boolean> activeWhen) {
            this.name = name;
            this.activeWhen = activeWhen;
        }
    
        /**
         * Sets the subsystem's state
         * @param s boolean true or false
         */
        public void setState(boolean s) {
            this.state = s;
        }
    
        /**
         * Gets whether the subsystem is active or not
         * @return true if the subsystem is active
         */
        public boolean isActive() {
            return this.activeWhen.get();
        }
    }
    
    /**
     * A static class with some possible pre-defined states for conditions
     */
    public static final class SubsystemConditionStates {
        public static final Supplier<Boolean> TELEOP = () -> DriverStation.getInstance().isOperatorControl();
        public static final Supplier<Boolean> AUTO = () -> DriverStation.getInstance().isAutonomous();
        public static final Supplier<Boolean> TELEOP_OR_AUTO = () -> TELEOP.get() || AUTO.get();
        public static final Supplier<Boolean> ALWAYS = () -> true;
    }
    
    private final HashMap<String, SubsystemCondition> conditions = new HashMap<>();
    
    /**
     * Creates a condition for later use
     * @param name The condition name
     * @param activeWhen A anonymous function representing when this condition's state should be considered
     */
    protected final void createCondition(String name, Supplier<Boolean> activeWhen) {
        conditions.put(name, new SubsystemCondition(name, activeWhen));
    }
    
    /**
     * Checks if the provided conditions (or all the conditions) are all true
     * @param conditions The list of conditions to check (All conditions will be checked if this is of length 0)
     * @return True if all checked conditions are good, false otherwise
     */
    protected final boolean checkConditions(String... conditions) {
        List<String> conditionArgs = List.of(conditions);
        
        // From all the conditions in this class
        return this.conditions.values().stream()
                // If there are any arguments to this function, use only those, or if there aren't any, use all
                .filter(x -> conditions.length == 0 || conditionArgs.contains(x.name))
                // Filter out any conditions that are inactive
                .filter(SubsystemCondition::isActive)
                // Return true if the conditions after being filtered are all true
                .allMatch(x -> x.state);
    }
    
    /**
     * Gets a condition by name
     * @param name The name of the condition
     * @return The condition with the name
     */
    public final SubsystemCondition getCondition(String name) {
        return this.conditions.get(name);
    }
    
    /**
     * TODO: Outputs some debug to the console.
     */
    public final void outputDebug() {
    
    }
}
