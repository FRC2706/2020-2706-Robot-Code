package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Supplier;

/**
 * A SubsystemOperationManager manages a list of operations required for this subsystem to run.
 * Commands and other subsystems can register operations and the owner of the manager can check if they are satisfied.
 */
public class SubsystemOperationManager {
    // A list of all the Operations for this operation manager
    private HashSet<SubsystemOperation> operations = new HashSet<>();
    
    // Name for debugging
    private String name;
    
    /**
     * Creates a new operation manager.
     * @param name The name of this manager for debugging
     */
    public SubsystemOperationManager(String name) {
        this.name = name;
    }
    
    /**
     * Creates a new operation and automatically registers it with this manager
     * @param name The name of the operation
     * @param applicable A function representing when this operation is applicable to the subsystem
     * @param startEnabled If this operation starts enabled
     * @return The new operation ready to use
     */
    public SubsystemOperation createOperation(String name, Supplier<Boolean> applicable, boolean startEnabled) {
        SubsystemOperation newOperation = new SubsystemOperation(name, applicable, startEnabled);
        operations.add(newOperation);
        return newOperation;
    }
    
    /**
     * Checks if all the operations are satisfied and this subsystem is allowed to run
     * @return true if all the operations that match the options are satisfied, false otherwise
     */
    public boolean allSatisfied() {
        // For all the operations, make sure the ones that match the provided options are enabled
        return operations.stream().noneMatch(c ->
                // Only match if the operation is false and the operation is applicable
                !c.getEnabled() && c.isApplicable()
        );
    }
    
    /**
     * This is for debug, it will send all the operations registered to the dashboard to help figure out why a
     * subsystem isn't running
     */
    public void sendOperationsToDashboard() {
        ArrayList<String> data = new ArrayList<>();
        for (var operation : operations) {
            data.add(operation.getName() + ": " + operation.getEnabled());
        }
        SmartDashboard.putStringArray(this.name + " operation", data.toArray(new String[0]));
    }
}
