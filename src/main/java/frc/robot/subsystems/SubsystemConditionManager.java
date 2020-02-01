package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.EnumSet;

/**
 * A SubsystemConditionManager manages a list of conditions required for this subsystem to run.
 * Commands and other subsystems can register conditions and the owner of the manager can check if they are satisfied.
 */
public class SubsystemConditionManager {
    // A list of all the conditions for this condition manager
    private ArrayList<SubsystemCondition> conditions = new ArrayList<>();
    
    // Name for debugging
    private String name;
    
    public SubsystemConditionManager(String name) {
        this.name = name;
    }
    
    /**
     * Creates a new condition and automatically registers it to the condition manager
     * @param name The name of the condition
     * @param options The options of the condition (Teleop? Auto? Both?)
     * @param startEnabled If this condition starts enabled
     * @return The new condition ready to use
     */
    public SubsystemCondition createCondition(String name, EnumSet<SubsystemCondition.ConditionOptions> options, boolean startEnabled) {
        SubsystemCondition newCondition = new SubsystemCondition(name, options, startEnabled);
        conditions.add(newCondition);
        return newCondition;
    }
    
    /**
     * Checks if all the conditions are satisfied and this subsystem is allowed to run
     * @param options The options to use to check the conditions (Teleop? Auto? Both?)
     * @return true if all the conditions that match the options are satisfied, false otherwise
     */
    public boolean allSatisfied(final EnumSet<SubsystemCondition.ConditionOptions> options) {
        // For all the conditions, make sure the ones that match the provided options are enabled
        return conditions.stream().noneMatch(c ->
                // Only match if the condition is false and the options apply
                !c.getEnabled() && c.getOptions().containsAll(options)
        );
    }
    
    /**
     * Overload for allSatisfied, but taking a boolean representing if this is auto (or teleop)
     * @param isAuto in auto?
     * @return true if all conditions that match the options are satisfied, false otherwise
     */
    public boolean allSatisfied(boolean isAuto) {
        var options = SubsystemCondition.ConditionOptions.fromIsAuto(isAuto);
        return allSatisfied(options);
    }
    
    /**
     * This is for debug, it will send all the conditions registered to the dashboard to help figure out why a
     * subsystem isn't running
     */
    public void sendConditionsToDriverStation() {
        ArrayList<String> data = new ArrayList<>();
        for (var condition : conditions) {
            data.add(condition.getName() + ": " + condition.getEnabled());
        }
        SmartDashboard.putStringArray(this.name + " conditions", data.toArray(new String[0]));
    }
}
