package frc.robot.subsystems;

import java.util.EnumSet;

public class SubsystemCondition {
    
    /**
     * A list of the possible options conditions can have
     */
    public enum ConditionOptions {
        ENABLED_FOR_TELEOP,
        ENABLED_FOR_AUTO;
        
        // Enum sets of the combinations for the options
        public static EnumSet<ConditionOptions> TELEOP = EnumSet.of(ENABLED_FOR_TELEOP);
        public static EnumSet<ConditionOptions> AUTO = EnumSet.of(ENABLED_FOR_AUTO);
        public static EnumSet<ConditionOptions> BOTH = EnumSet.of(ENABLED_FOR_TELEOP, ENABLED_FOR_AUTO);
    
        /**
         * Helper method to get the appropriate EnumSet from an 'isAuto' boolean
         * @param isAuto Boolean representing if we're in auto. (or teleop if false)
         * @return The EnumSet representing the options
         */
        public static EnumSet<ConditionOptions> fromIsAuto(boolean isAuto) {
            return isAuto ? SubsystemCondition.ConditionOptions.AUTO : SubsystemCondition.ConditionOptions.TELEOP;
        }
    }
    
    // The options used to create this condition
    private EnumSet<ConditionOptions> options;
    
    // The name given to this condition
    private String name;
    
    // The state of this condition
    private boolean enabled;
    
    /**
     * Constructor for a condition. DO NOT USE THIS TO MAKE A CONDITION YOURSELF.
     * Use the {@code SubsystemConditionManager.createCondition} method instead!
     * @param name The name of the condition
     * @param options The options for this condition
     * @param startEnabled The start state of this condition
     */
    public SubsystemCondition(String name, EnumSet<ConditionOptions> options, boolean startEnabled) {
        this.name = name;
        this.options = options;
        this.enabled = startEnabled;
    }
    
    /**
     * Setter for if this condition is enabled
     * @param enabled enabled?
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    
    /**
     * @return is this condition is enabled?
     */
    public boolean getEnabled() {
        return this.enabled;
    }
    
    /**
     * @return This condition's name
     */
    public String getName() {
        return this.name;
    }
    
    /**
     * @return This condition's options
     */
    public EnumSet<ConditionOptions> getOptions() {
        return this.options;
    }
}
