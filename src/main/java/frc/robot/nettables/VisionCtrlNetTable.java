package frc.robot.nettables;

//for NetworkTable
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Config;

public class VisionCtrlNetTable {

  //For NetworkTables
  //Get the default instance of NetworkTables that was created automatically
  //when your program starts
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  //Add a visionControl network table
  private NetworkTable visionControlTable;
  private NetworkTableEntry VisionShutDownEntry;
  private NetworkTableEntry VisionStartUpEntry;
  private NetworkTable mergeVisionTable;

  private double distanceToPowerCell;
  private double yawToPowerCell;

  public VisionCtrlNetTable ()
  {
    visionControlTable = inst.getTable("VisionControl");
    VisionShutDownEntry = visionControlTable.getEntry("ShutDown");
    VisionStartUpEntry = visionControlTable.getEntry("StartUp");

    VisionShutDownEntry.setBoolean(false);
    VisionStartUpEntry.setBoolean(false);

    mergeVisionTable = inst.getTable(Config.VISION_TABLE_NAME);
    distanceToPowerCell = mergeVisionTable.getEntry(Config.DISTANCE_POWERCELL).getDouble(-1);
    yawToPowerCell = mergeVisionTable.getEntry(Config.YAW_POWERCELL).getDouble(-1);

    



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