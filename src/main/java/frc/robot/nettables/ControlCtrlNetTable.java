package frc.robot.nettables;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ControlCtrlNetTable {

    //For NetworkTables
    //Get the default instance of NetworkTables that was created automatically
    //when your program starts
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();

    //todo: figure if Control pi needs a start signal
    //Add a new network table for the control system.
    private NetworkTable ControlShutdown_table; 
    private NetworkTableEntry ControlShutDownEntry; 

    public ControlCtrlNetTable ()
    {
        ControlShutdown_table = instance.getTable("ControlShutDown");
        ControlShutDownEntry = ControlShutdown_table.getEntry("ShutDown");
        ControlShutDownEntry.setBoolean(false);
    }

    /**
     * Writes a shut down signal to the controls network table.
     *
     */
    public void shutDownControl()
    {
        ControlShutDownEntry.setBoolean(true);
    }
}