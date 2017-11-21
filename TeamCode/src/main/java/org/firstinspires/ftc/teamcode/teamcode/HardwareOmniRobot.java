package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.DecimalFormat;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Tile Runner.  The Tile Runner is setup with two motors
 * on each side.  This version assumes each motor is connected to an individual port.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor1:        "left_motor1"
 * Motor channel:  Left  drive motor2:        "left_motor2"
 * Motor channel:  Right drive motor1:        "right_motor1"
 * Motor channel:  Right drive motor2:        "right_motor2"
 */
public class HardwareOmniRobot
{
    ElapsedTime runtime = new ElapsedTime();

    ColorSensor jkcolor, jkcolor2;

    ModernRoboticsI2cGyro gyro;

    ModernRoboticsI2cRangeSensor ultra_front, ultra_right, ultra_left, ultra_back;

    public final int GRABBER_AUTOPOS = 1300;

    //change
    /* Public OpMode members. */
    public AnalogInput flex = null;
    public DcMotor leftMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor rightMotor2 = null;
    public DcMotor wheelie = null;
    public DcMotor grabber = null;
    public Servo jknock = null;
    public DcMotor dumper = null;
    // public Servo grabber = null;
    public Servo claw1 = null;
    public Servo claw2 = null;
    //public DcMotor reel = null;
    //public DcMotor slide = null;
    //public Servo wrist = null;
    //public Servo clamp = null;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    public static final String MESSAGETAG = "5040MSG";

    /* start FLEX SENSOR instance fields */
    private int columnNum = 0;
    private double flexCurrent;
    private double flexPrevious = 0;
    private final double TOLERANCE = 0.30;
    /* end FLEX SENSOR instance fields */

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmniRobot(){

        hwMap = null;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean rungyro) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //try {
        leftMotor1 = hwMap.dcMotor.get("left_motor1");
        leftMotor2 = hwMap.dcMotor.get("left_motor2");
        rightMotor1 = hwMap.dcMotor.get("right_motor1");
        rightMotor2 = hwMap.dcMotor.get("right_motor2");
        wheelie = hwMap.dcMotor.get("wheelie");
        grabber = hwMap.dcMotor.get("grabber");
        //slide = hwMap.dcMotor.get("slide");
        //reel = hwMap.dcMotor.get("reel");
        dumper = hwMap.dcMotor.get("dumper");
        claw1 = hwMap.servo.get("claw_1");
        claw2 = hwMap.servo.get("claw_2");
        jknock = hwMap.servo.get("jknock");
        //wrist = hwMap.servo.get("wrist");
        //clamp = hwMap.servo.get("clamp");
        jkcolor = hwMap.get(ColorSensor.class, "color_sense");
        jkcolor2 = hwMap.get(ColorSensor.class, "color");

        jkcolor2.setI2cAddress(I2cAddr.create8bit(0x28));

        ultra_back = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_back");
        //ultra_front = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_front");
        ultra_left = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_left");
        ultra_right = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_right");

        //ultra_front.setI2cAddress(I2cAddr.create8bit(0x10));
        ultra_left.setI2cAddress(I2cAddr.create8bit(0x12));
        ultra_right.setI2cAddress(I2cAddr.create8bit(0x14));
        ultra_back.setI2cAddress(I2cAddr.create8bit(0x16));


        flex = hwMap.analogInput.get("flx");

        //reel.setDirection(DcMotor.Direction.FORWARD);
        //slide.setDirection(DcMotor.Direction.REVERSE);
        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //grabber.setDirection(Servo.Direction.REVERSE);
        grabber.setDirection(DcMotor.Direction.REVERSE);
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dumper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set all motors to zero power
        leftMotor1.setPower(0);
        rightMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);
        wheelie.setPower(0);
        //slide.setPower(0);
        //reel.setPower(0);
        jknock.setPosition(0.7);
        claw1.setPosition(0.0);
        claw2.setPosition(1.0);
        //wrist.setPosition(0);
        //grabber.scaleRange(0.0, 0.25);
        //grabber.setPosition(0.220);

        //grabber.setPower(0.75);
        //grabber.setTargetPosition(1485);

        dumper.setPower(0);

        if(rungyro == true) {
            gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
            gyro.calibrate();
        }

    }

    public void RotateTo(int degrees) {
        int heading = gyro.getHeading();
        while(heading != degrees)
            if(degrees < heading) {
                onmiDrive(0.0,0.0,-0.7);
            }
            else {
                onmiDrive(0.0,0.0,0.7);
            }
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    /***
     * onmiDrive is for the teleop drive part of the competition
     * Takes input of the joysticks

     */

    //rx *= -1;
    //ry *= -1;
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    /*public void relicLifter(boolean dup, boolean ddown, boolean dleft, boolean dright) {
        if ((dup == true) && (ddown == false)) {
            slide.setPower(-90);
        }
        else if ((dup == false)&&(ddown == true)) {
            slide.setPower(90);

        }
        else {
            slide.setPower(0);
        }

        if ((dleft == true)&&(dright == false)) {
            reel.setPower(-75);
        }
        else  if ((dleft == false)&&(dright== true)) {
            reel.setPower(75);
        }
        else {

            reel.setPower(-10);
        }

    }*/


    public void onmiDrive (double sideways, double forward, double rotation)
    {
        try {
            leftMotor1.setPower(limit(((forward - sideways)/2) * 1 + (-.25 * rotation)));
            leftMotor2.setPower(limit(((forward + sideways)/2) * 1 + (-.25 * rotation)));
            rightMotor1.setPower(limit(((-forward - sideways)/2) * 1 + (-.25 * rotation)));
            rightMotor2.setPower(limit(((-forward + sideways)/2) * 1 + (-.25 * rotation)));
        } catch (Exception e) {
            RobotLog.ee(MESSAGETAG, e.getStackTrace().toString());
        }
    }
}
