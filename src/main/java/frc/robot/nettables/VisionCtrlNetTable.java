package frc.robot.nettables;

//for NetworkTable
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionCtrlNetTable {

  //For NetworkTables
  //Get the default instance of NetworkTables that was created automatically
  //when your program starts
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  //Add a visionControl network table
  private NetworkTable VisionControl_table ;
  private NetworkTableEntry VisionShutDownEntry ;
  private NetworkTableEntry VisionStartUpEntry ;

  public VisionCtrlNetTable ()
  {
    VisionControl_table = inst.getTable("VisionControl");
    VisionShutDownEntry = VisionControl_table.getEntry("ShutDown");
    VisionStartUpEntry = VisionControl_table.getEntry("StartUp");

    VisionShutDownEntry.setBoolean(false);
    VisionStartUpEntry.setBoolean(false);
  }

  /**
  * Writes a shut down signal to the vision control table.
  *
  */
  public void shutDownVision()
  {
    VisionShutDownEntry.setBoolean(true);
    VisionStartUpEntry.setBoolean(false);

  }
  
  /**
  * Writes a start up signal to the vision control table.
  *
  */
  public void startUpVision()
  {
    VisionShutDownEntry.setBoolean(false);
    VisionStartUpEntry.setBoolean(true);

  }


}