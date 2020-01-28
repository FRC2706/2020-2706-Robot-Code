package frc.robot.sensors;
import com.ctre.phoenix.sensors.*;
import frc.robot.subsystems.DriveBase;

public class Pigeon {

    static PigeonIMU _pidgey;      // Pigeon IMU used to enforce straight drive

    static {
        //get talon
        _pidgey = new PigeonIMU(DriveBase.getInstance().leftFrontTalon);

        //setup gyro angle finder
        _pidgey.setFusedHeading(0.0, 30);
    }

    public static PigeonIMU getPigeon() {
        return _pidgey;
    }

    public static double getCurrentAngle(){
        //Gets the current angle
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
        _pidgey.getFusedHeading(fusionStatus);
        return fusionStatus.heading;
    }
}
