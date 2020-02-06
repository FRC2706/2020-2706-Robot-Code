/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Code by Charlie Macdonald.
 * Last modified on Feb 5th, 2020.
 */

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class IrSensor implements Sendable {
    private static final int kIRPort = 1; //set IR sensor port
        //TODO: The IR sensor's port may change, depending on the robot setup. (Use a robot-specific config to set this?)

    //median filter to discard outliers. Filters over 10 samples
    private final MedianFilter m_filter = new MedianFilter(10);

    private final AnalogInput m_irsensor;

    //Set the formula values to calculate IR sensor distance:
        //Values for short range (5-30cm) sensor: 12.8528, -0.007545, -3.92, 35, 4.5.
        //TODO: If a different IR sensor is used, these values will need to be updated. (Use a robot-specific config to set them?)
    private final double irSlopeValue = 12.8528;
    private final double irXValue = -0.007545;
    private final double irConstant = -3.92;
    private final double irMaxDistance = 35;
    private final double irMinDistance = 4.5;
  
    public double getDistance() {
        //Calculate distance (cm) from mV:
        double currentDistance = ((1/(((m_filter.calculate(m_irsensor.getValue()))/1000 +irXValue)/irSlopeValue))+irConstant);
        //Accurate within +/- 1.5cm when measuring from 5-20cm.

        //The following "if" statements fix the values outside the sensor's accurate measuring range.
        if (currentDistance > irMaxDistance) {
            currentDistance = 35; //set distance to a fixed value if it is above the max measuring range.
        }
        if (currentDistance < irMinDistance) {
            currentDistance = 0; //set distance to a fixed value if it is below the min measuring range.
            //Warning: Distances under 2cm will still produce erroneous values.
        }

        return(currentDistance);

    }

    public IrSensor(int channel) {
        this.m_irsensor = new AnalogInput(kIRPort);
        SendableRegistry.addChild(this, m_irsensor);
    }

	@Override
	public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Distance", this::getDistance, null);
	}
}
