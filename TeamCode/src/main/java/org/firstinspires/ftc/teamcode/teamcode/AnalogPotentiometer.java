package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This routine converts the voltage from the an analog input
 * which is connected to potentiometer into a value which is
 * useful by the program.
 * Inputs:
 *    - potentiometer - AnalogInput object
 *    - maxValue - maximum value of the returned values
 *    - offset - offset of the maxValue
 *
 * Example:  Need a range of values between -90 and 90 degrees.
 * Enter 180 as the maxValue to give you a 180 degrees and then
 * -90 for the offset.
 */

public class AnalogPotentiometer {

    private final double MAXVOLTAGE = 5;
    private final String MSG5040 = "5040MSG";
    private AnalogInput potentiometer =null;
    private double maxValue = MAXVOLTAGE;
    private double offset = 0;


    AnalogPotentiometer(AnalogInput potentiometer, double maxValue, double offset){
        this.potentiometer = potentiometer;
        this.maxValue = maxValue;
        this.offset = offset;
    }

    AnalogPotentiometer(AnalogInput potentiometer){
        this.potentiometer = potentiometer;
    }

    public double getValue(){
        double conversion = 0;

        try {
            conversion = (potentiometer.getVoltage() * maxValue / MAXVOLTAGE) + offset;
        } catch (Exception e) {
            RobotLog.ee(MSG5040, e.toString());
        }

        return conversion;
    }

    public double getValueRaw(){
        return potentiometer.getVoltage();
    }

    public String toString(){
        return Double.toString(getValue());
    }
}
