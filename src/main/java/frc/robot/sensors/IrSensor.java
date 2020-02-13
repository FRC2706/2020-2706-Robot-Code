/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class IrSensor implements Sendable {
    //Make a median filter to discard outliers from measurements. Filters over 10 samples
    private final MedianFilter m_filter = new MedianFilter(10);

    private final AnalogInput m_irsensor;

    //Set the constants to calculate IR sensor distance:
        //Values for short range (Sharp GP2Y0A41SK0F) 4-30cm sensor: 12.8528, 0.007545, -3.92, 25, 4.5, 0.
        //Note: If a different IR sensor is used, these values will need to be updated.
    private final double IR_SLOPE_VALUE = 12.8528;
    private final double IR_X_VALUE = 0.007545;
    private final double IR_CONSTANT = -3.92;
    private final double IR_MAX_DISTANCE_CM = 25;
    private final double IR_MIN_DISTANCE_CM = 4.5;
    private final double IR_MIN_OUTPUT_CM = 0.0;

    private double irVoltage;
  
    public double takeMeasurement() {
        //Make sure to call this method in the periodic() method of the subsystem that uses the IR sensor. 
        //This method continuously monitors the IR sensor output and filters outliers. Also returns sensor voltage if anyone wants to use that.
        
        irVoltage = (m_filter.calculate(m_irsensor.getValue())/1000); 
        //Gets IR sensor value, filters outliers, and converts milivolt output to volts.

        return(irVoltage);
    }

    public double getDistance() {
        //Calculate distance (cm) from mV:
        //Accurate to +/- 2cm when measuring within 4.5-25cm.
        double currentDistanceCm = ((1/
            ((irVoltage - IR_X_VALUE)
            /IR_SLOPE_VALUE)
            ) //Inverse is effectively calculated at this point.
            +IR_CONSTANT);

        //These statements limit currentDistanceCm to a predetermined number when the distance is outside the sensor's accurate range.
        //Warning: Distances under 1cm will still produce erroneous measurements, regardless of the following code.
        if (currentDistanceCm > IR_MAX_DISTANCE_CM) {
            currentDistanceCm = IR_MAX_DISTANCE_CM; //set distance to the maximum distance if the measured value goes above the max distance.
        }
        else if (currentDistanceCm < IR_MIN_DISTANCE_CM) {
            currentDistanceCm = IR_MIN_OUTPUT_CM; //set distance to the minimum output if the measured value falls below the min distance.
        }

        return(currentDistanceCm);
    }

    public IrSensor(int channel) {
        this.m_irsensor = new AnalogInput(channel);
        SendableRegistry.addChild(this, m_irsensor);
    }

	@Override
	public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Voltage", this::takeMeasurement, null);
    }
}
