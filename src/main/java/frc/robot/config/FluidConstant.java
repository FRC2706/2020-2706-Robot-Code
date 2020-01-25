package frc.robot.config;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.function.BiConsumer;

public class FluidConstant<T> {
    private T value;
    private final T initialValue;
    private final String name;
    private final ArrayList<BiConsumer<T, T>> updateActions = new ArrayList<>();


    /**
     * Create a new FluidConstant of type T
     * @param name The name to be used in NetworkTables and files
     * @param defaultValue The compile-time default value
     */
    public FluidConstant(String name, T defaultValue) {
        this.value = defaultValue;
        this.initialValue = defaultValue;
        this.name = name;
    }

    /**
     * Registers this constant to an entry in the given table
     * @param table The NetworkTable to bind this constant to
     * @return The instance of this FluidConstant
     */
    public FluidConstant<T> registerToTable(NetworkTable table) {
        // Register the entry and set the value if it doesn't exist
        NetworkTableEntry ntEntry = table.getEntry(name);
        if (!ntEntry.exists()) ntEntry.setValue(this.value);
        ntEntry.addListener(this::entryUpdated, EntryListenerFlags.kUpdate);
        // Return the instance to make one-line declarations possible
        return this;
    }

    /**
     * Registers an update listener to this constant
     * @param updateAction The action to run on an update
     * @return The instance of this FluidConstant
     */
    public FluidConstant<T> registerListener(BiConsumer<T, T> updateAction) {
        updateActions.add(updateAction);
        // Return the instance to make one-line decelerations possible
        return this;
    }

    /**
     * @return The current value of this constant
     */
    public T getValue() {
        return this.value;
    }

    /**
     * @return The initial value of this constant (at compile-time)
     */
    public T getInitialValue() {
        return this.initialValue;
    }

    /**
     * Private NetworkTableEntry listener method for when the entry is updated
     * @param notification info about the notification
     */
    private void entryUpdated(EntryNotification notification) {
        // Only allow the value to be updated while the robot is disabled
        if (DriverStation.getInstance().isDisabled()) return;
        // This is a safe cast because NetworkTables already prevents assigning different types to NTEntries
        T newValue = (T) notification.value.getValue();
        // Send the update to all the registered actions
        this.updateActions.forEach(a -> a.accept(this.value, newValue));
        this.value = newValue;
    }
}
