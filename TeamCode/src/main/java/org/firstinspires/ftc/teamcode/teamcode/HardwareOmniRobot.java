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

    ModernRoboticsI2cRangeSensor ultra_right, ultra_left, ultra_back;

    public final int GRABBER_AUTOPOS = 1395;
    public final double JKUP = 0.8;

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
    public Servo claw1 = null;
    public Servo claw2 = null;
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
        RobotLog.ii("5040MSGHW","Motors gotten");
        wheelie = hwMap.dcMotor.get("wheelie");
        grabber = hwMap.dcMotor.get("grabber");
        dumper = hwMap.dcMotor.get("dumper");
        claw1 = hwMap.servo.get("claw_1");
        claw2 = hwMap.servo.get("claw_2");
        jknock = hwMap.servo.get("jknock");
        jkcolor = hwMap.get(ColorSensor.class, "color_sense");
        jkcolor2 = hwMap.get(ColorSensor.class, "color");
        RobotLog.ii("5040MSGHW","Everything but ultras gotten");

        jkcolor2.setI2cAddress(I2cAddr.create8bit(0x28));

        ultra_back = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_back");
        ultra_left = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_left");
        ultra_right = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_right");

        ultra_left.setI2cAddress(I2cAddr.create8bit(0x12));
        ultra_right.setI2cAddress(I2cAddr.create8bit(0x14));
        ultra_back.setI2cAddress(I2cAddr.create8bit(0x16));
        RobotLog.ii("5040MSGHW","Everything set up");

        flex = hwMap.analogInput.get("flx");

        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        grabber.setDirection(DcMotor.Direction.REVERSE);
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dumper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RobotLog.ii("5040MSGHW","Everything setMode adn Direction run");

        // Set all motors to zero power
        leftMotor1.setPower(0);
        rightMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor2.setPower(0);
        RobotLog.ii("5040MSGHW","Drive Train setPower");
        wheelie.setPower(0);
        jknock.setPosition(0.8);
        claw1.setPosition(0.0);
        claw2.setPosition(1.0);
        dumper.setPower(0);

        if(rungyro == true) {
            gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
            gyro.calibrate();
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
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }


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
