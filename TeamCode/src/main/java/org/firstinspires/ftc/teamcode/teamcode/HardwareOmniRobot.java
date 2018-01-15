package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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

    ColorSensor jkcolor, jkcolor2, dumperColor;

    ModernRoboticsI2cGyro gyro, gyro2;
    BNO055IMU imu;

    ModernRoboticsI2cRangeSensor ultra_backMR, ultra_backMR2;

    public final int GRABBER_AUTOPOS = 1395;
    public final double JKUP = 0.8;

    /* Public OpMode members. */
    public AnalogInput flex = null;
    public AnalogInput ultra_left = null;
    public AnalogInput ultra_back = null;
    public AnalogInput ultra_right = null;
    public DcMotor leftMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor rightMotor2 = null;

    public DcMotor relicMotor = null;
    public Servo relicWrist   = null;
    public Servo relicClaw    = null;
    public Servo relicStopper = null;

    public DcMotor wheelie = null;
    public DcMotor grabber = null;
    public Servo jknock = null;
    public DcMotor dumper = null;
    public Servo claw1 = null;
    public Servo claw2 = null;
    public Servo jewelGrab = null;
    public Servo flexServo = null;
    public Servo glyphStop = null;
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
        jewelGrab = hwMap.servo.get("JewelGrab");
        claw1 = hwMap.servo.get("claw_1");
        claw2 = hwMap.servo.get("claw_2");
        jknock = hwMap.servo.get("jknock");
        flexServo = hwMap.servo.get("flex");
        jkcolor = hwMap.get(ColorSensor.class, "color_sense");
        jkcolor2 = hwMap.get(ColorSensor.class, "color");
        RobotLog.ii("5040MSGHW","Everything but ultras gotten");

        jkcolor2.setI2cAddress(I2cAddr.create8bit(0x28));

        ultra_backMR = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_backMR");
        ultra_backMR2 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_backMR2");

        relicMotor = hwMap.dcMotor.get("relic_motor");
        relicMotor.setDirection(DcMotor.Direction.REVERSE);
        relicMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicWrist = hwMap.get(Servo.class, "relic_wrist");
        relicClaw = hwMap.get(Servo.class, "relic_claw");
        relicStopper = hwMap.get(Servo.class, "extension_stopper");

        //ultra_back = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_back");
        //ultra_left = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_left");
        //ultra_right = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultra_right");

        //ultra_left.setI2cAddress(I2cAddr.create8bit(0x12));
        //ultra_right.setI2cAddress(I2cAddr.create8bit(0x14));
        ultra_backMR.setI2cAddress(I2cAddr.create8bit(0x16));
        ultra_backMR2.setI2cAddress(I2cAddr.create8bit(0x14));
        RobotLog.ii("5040MSGHW","Everything set up");

        ultra_left = hwMap.analogInput.get("ultra_left");
        ultra_back = hwMap.analogInput.get("ultra_back");
        ultra_right = hwMap.analogInput.get("ultra_right");

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

        RobotLog.ii("5040MSGHW","Everything setMode and Direction run");

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
        jewelGrab.setPosition(0.19);
        dumper.setPower(0);
        relicClaw.setPosition(0.0);
        relicWrist.setPosition(0.94);
        relicStopper.setPosition(0.98);
        flexServo.setPosition(0.196);        //out to 90 -- 0.82
        RobotLog.ii("5040MSGHW", "Everything Initialized Correctly");


        if(rungyro == true) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            this.imu = hwMap.get(BNO055IMU.class, "imu");
            this.imu.initialize(parameters);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            //Close the claw to clear the relic arm
            claw2.setPosition(0.3);

            //Move the grabber Up
            while(grabber.getCurrentPosition() < GRABBER_AUTOPOS - 10) {
                grabber.setPower(0.75);
                grabber.setTargetPosition(GRABBER_AUTOPOS);
            }

            //Move the claw back to a semi-open position
            claw2.setPosition(0.9);
            relicClaw.setPosition(0.35);
            //The robot is now initialized within 18 inches!
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
            leftMotor1.setPower(limit(((forward - sideways)/2) * 1 + (-.2 * rotation)));
            leftMotor2.setPower(limit(((forward + sideways)/2) * 1 + (-.2 * rotation)));
            rightMotor1.setPower(limit(((-forward - sideways)/2) * 1 + (-.2 * rotation)));
            rightMotor2.setPower(limit(((-forward + sideways)/2) * 1 + (-.2 * rotation)));
        } catch (Exception e) {
            RobotLog.ee(MESSAGETAG, e.getStackTrace().toString());
        }
    }

    public void relicArm (double ljoystick, double rjoystick, boolean a){
        int relicMotorPosition = relicMotor.getCurrentPosition();
        int newRelicMotorPosition = relicMotorPosition;

        double rwCurrent = relicWrist.getPosition(), rwGoal = rwCurrent;

        double power = 0.5;
        final int RELIC_OUT = 4800; // Minimum Value to Prevent Over Extension
        final int RELIC_IN  = 0;

        if(ljoystick > 0.1){
            newRelicMotorPosition = RELIC_OUT;
        }else if(ljoystick < -0.1){
            newRelicMotorPosition = RELIC_IN;
        }else{
            newRelicMotorPosition = relicMotorPosition;
        }

        if(rjoystick > 0.1){
            rwGoal = rwCurrent += 0.1;
        }else if(rjoystick < -0.1){
            rwGoal = rwCurrent -= 0.1;
        }
        if(rwGoal > 1.0){
            rwGoal = 1.0;
        }else if(rwGoal < 0.0){
            rwGoal = 0.0;
        }

        relicWrist.setPosition(rwGoal);

        /*
        if(rjoystick > -0.1){
            if(rwCurrent + 0.05 <= 1.0){
                rwGoal = rwCurrent + 0.05;
            }
        }else if(rjoystick > 0.1) {
            if (rwCurrent - 0.05 >= 0) {
                rwGoal = rwCurrent - 0.05;
            }
        }
        */

        relicMotor.setTargetPosition(newRelicMotorPosition);
        relicMotorPosition = relicMotor.getCurrentPosition();
        power = Math.abs(ljoystick);

        if(power < 0.4){
            power = 0.4;
        }

        //relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry.addData("Right Joystick Value", joystick);
        //telemetry.addData("Motor Target Position", newRelicMotorPosition);
        //telemetry.addData("Motor Power Value", power);

        /*if(relicMotor.isBusy()) {
            relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            relicMotor.setPower(power);
        }else{
            //relicMotor.setPower();
            relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            relicMotor.setTargetPosition(relicMotor.getCurrentPosition());
        }*/

        relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicMotor.setPower(power);

        //relicWrist.setPosition(rwGoal);
        //waitForTick(50);
    }




}
