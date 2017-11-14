package org.firstinspires.ftc.teamcode.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public navXPIDController yawPIDController;
    public navXPIDController.PIDResult yawPIDResult;
    ElapsedTime runtime = new ElapsedTime();

    ColorSensor jkcolor, jkcolor2;

    ModernRoboticsI2cRangeSensor ultra_front, ultra_right, ultra_left, ultra_back;

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
    private final int NAVX_DIM_I2C_PORT = 0;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public AHRS navx_device;

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
    public void init(HardwareMap ahwMap) {
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
        //grabber = hwMap.servo.get("grabber");
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

        grabber.setPower(0.75);
        grabber.setTargetPosition(1485);

        dumper.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //reel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Enable NavX Sensor

        navx_device = AHRS.getInstance(hwMap.deviceInterfaceModule.get("DIM"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
       /* } catch (Exception e) {

            RobotLog.ee(MESSAGETAG,e.getMessage());

        }*/
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

    public void DriveFor(double time, double forward, double side, double rotate) {

        onmiDrive(side, -forward, rotate); //starts moving in wanted direction
        runtime.reset(); //resets time

        while (runtime.seconds() < time) {    //runs for amount of time wanted
        }
        onmiDrive(0.0, 0.0, 0.0); //stops  moving after
    }

    //Jewel knocking off code
    public void TurnLeft() {
        DriveFor(0.3, 0.0, 0.0, -0.5);
        jknock.setPosition(0.7);
        DriveFor(0.3, 0.0, 0.0, 0.5);
    }

    public void TurnRight() {
        DriveFor(0.3, 0.0, 0.0, 0.5);
        jknock.setPosition(0.7);
        DriveFor(0.3, 0.0, 0.0, -0.5);
    }

    public void JewelKnock(String side) {

        jknock.setPosition(0.0);
        jkcolor.enableLed(true);
        jkcolor2.enableLed(true);
        DriveFor(1.0,0.0,0.0,0.0);
        //while(jknock.getPosition() != 0.45){jknock.setPosition(0.0);}
        boolean decided = false;
        runtime.reset();
        int color1b = jkcolor.blue();
        int color1r = jkcolor.red();
        int color2b = jkcolor2.blue();
        int color2r = jkcolor2.red();

        while (decided == false && runtime.seconds() < 2) {
            if (color1r < 2 && color1b< 2 && color2r < 2 && color2b < 2) {
                decided = true;
                jknock.setPosition(1.2);
            }
            else if(side == "blue") {
                if((color1b>=2 && color1r<2) || (color2b<2 && color2r>=2)) {
                    TurnLeft();
                    decided = true;
                }
                else if((color1b<2 && color1r>=2) || (color2b>=2 && color2r<2)) {
                    TurnRight();
                    decided = true;
                }
            }
            else if(side == "red") {
                if((color1b>=2 && color1r<2) || (color2b<2 && color2r>=2)) {
                    TurnRight();
                    decided = true;
                }
                else if((color1b<2 && color1r>=2) || (color2b>=2 && color2r<2)) {
                    TurnLeft();
                    decided = true;
                }
            }
        }
        jkcolor.enableLed(false);
        jkcolor2.enableLed(false);
    }

    //Use to set angle wanted to move resets gyro when you do this ---- use navx_device.zeroYaw(); to zero it
    public void NavXInit(double TARGET_ANGLE_DEGREES) {
        //sets wanted angle degrees and other variables for first time run
        DecimalFormat df = new DecimalFormat("#.##");
        yawPIDController = new navXPIDController( navx_device, navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 1.0);
        yawPIDController.setPID(0.015, 0.0, 0.0);
        yawPIDController.enable(true);
        yawPIDResult = new navXPIDController.PIDResult();
    }

    //Used to move to desired angle while moving or just turning in place
    public void NavX(double forward, double side) {
        //when running it turns to position
        //yawPIDResult = new navXPIDController.PIDResult();
        if (yawPIDController.isNewUpdateAvailable(new navXPIDController.PIDResult())) {
            if (yawPIDResult.isOnTarget()) {
                onmiDrive(side,-forward,0);
            } else {
                double output = yawPIDResult.getOutput();
                onmiDrive(side,-forward,output);
            }
        }
    }

    public int getColumnNum(){


        flexCurrent = flex.getVoltage();

        if (flexPrevious - TOLERANCE > flexCurrent) {
            columnNum++;
        }
        flexPrevious = flexCurrent;
        return columnNum;
    }

    public int Vuforia(int cameraMonitorViewId, String side) {

        int choosen = 0;

        try {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AUBrQCz/////AAAAGXg5njs2FEpBgEGX/o6QppZq8c+tG+wbAB+cjpPcC5bwtGmv+kD1lqGbNrlHctdvrdmTJ9Fm1OseZYM15VBaiF++ICnjCSY/IHPhjGW9TXDMAOv/Pdz/T5H86PduPVVKvdGiQ/gpE8v6HePezWRRWG6CTA21itPZfj0xDuHdqrAGGiIQXcUbCTfRAkY7HwwRfQOM1aDhmeAaOvkPPCnaA228iposAByBHmA2rkx4/SmTtN82rtOoRn3/I1PA9RxMiWHWlU67yMQW4ExpTe2eRtq7fPGCCjFeXqOl57au/rZySASURemt7pwbprumwoyqYLgK9eJ6hC2UqkJO5GFzTi3XiDNOYcaFOkP71P5NE/BB    ";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            relicTrackables.activate();
            runtime.reset();
            while (choosen == 0 && runtime.seconds() < 3) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if(side == "red") {
                        switch (vuMark) {
                            case LEFT:
                                choosen = 1;
                                break;
                            case CENTER:
                                choosen = 2;
                                break;
                            case RIGHT:
                                choosen = 3;
                                break;
                        }
                    }
                    else {
                        switch (vuMark) {
                            case LEFT:
                                choosen = 3;
                                break;
                            case CENTER:
                                choosen = 2;
                                break;
                            case RIGHT:
                                choosen = 1;
                                break;
                        }
                    }
                }
            }
        }catch (Exception e){
            choosen = 0;
        }

        return choosen;
    }
}
