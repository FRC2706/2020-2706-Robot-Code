package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Type;
import java.time.Instant;
import java.util.Date;
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
    protected final SubsystemCondition createCondition(String name, Supplier<Boolean> activeWhen) {
        var condition = new SubsystemCondition(name, activeWhen);
        conditions.put(name, condition);
        return condition;
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
     * Outputs debug info about this subsystem's conditions including:
     * A timestamp, the state of this subsystem, the DriverStation state and
     * all the conditions with their states and active statuses
     */
    public final void outputDebug() {
        StringBuilder output = new StringBuilder(String.format("Conditional subsystem debug for %s:\n" +
                        "\tTime: %tc\n" +
                        "\tSubsystem State: Active(%s)\n" +
                        "\tDS State: Teleop(%s), Auto(%s), Test(%s)\n" +
                        "\tConditions:\n", this.getClass().getSimpleName(), Date.from(Instant.now()),
                checkConditions(),
                DriverStation.getInstance().isOperatorControl(),
                DriverStation.getInstance().isAutonomous(),
                DriverStation.getInstance().isTest()));
    
        for (var condition : this.conditions.values()) {
            output.append(String.format("\t\t[%s] State(%s), Active(%s)", condition.name,
                    condition.state, condition.activeWhen.get()));
        }
        
        System.out.println(output);
    }
}
