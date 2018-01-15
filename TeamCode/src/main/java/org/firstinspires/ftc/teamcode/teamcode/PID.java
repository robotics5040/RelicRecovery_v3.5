package org.firstinspires.ftc.teamcode.teamcode;

/**
 * Created by jedch on 1/14/2018.
 */

public class PID {

    private double p;
    private double i;
    private double d;

    private double setPoint;
    private double error;
    private double i_error;

    private long time;

    public PID(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;

        this.setPoint = 0;
        this.error = 0;
        this.i_error = 0;

        this.time = System.currentTimeMillis();
    }

    public void setSetPoint(double setpoint){
        this.setPoint = setpoint;
        this.error = 0;
        this.i_error = 0;
        this.time = System.currentTimeMillis();
    }

    public double secondsElapsed(){
        return (System.currentTimeMillis() - time + 1) / 1000;
    }

    public double update(double heading){
        double currentError = setPoint - heading;

        i_error += currentError * secondsElapsed();
        double errorDifference = currentError - error;
        errorDifference /= secondsElapsed();
        error = currentError;

        return p * currentError + i * i_error + d * errorDifference;
    }

}
