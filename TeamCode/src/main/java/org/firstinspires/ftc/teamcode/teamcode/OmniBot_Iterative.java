/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.teamcode;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Omnibot Pushbot", group="Pushbot")
//@Disabled
public class OmniBot_Iterative extends OpMode {
    private double position = 0.0;
        public int  pressed = 0;
        double wrist_num = 0;
        boolean aPressed=false,bPressed=false,xPressed=false,yPressed=true,closed = true;
        ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    private HardwareOmniRobot robot; // use the class created to define a Pushbot's hardware

    public OmniBot_Iterative() {
        robot = new HardwareOmniRobot();
    }

    // could also use HardwarePushbotMatrix class.
   /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.navx_device.zeroYaw();
        robot.NavXInit(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left_stick_x, left_stick_y,right_stick_x, right_stick_y, power, left_trigger, right_trigger,LX,RX,front=0,side=0;
        boolean NavXTemp, b_button1,a_button1,y_button1,x_button1,left_bumper, right_bumper, a_button, b_button, x_button, y_button,dup,ddown,dleft,dright,left_bump1,right_bump1, d_up1,d_down1,d_left1,d_right1,stick_press, stick_press1;


        //note: The joystick goes negative when pushed forwards, so negate it)
        left_stick_x = gamepad1.left_stick_x;
        left_stick_y = gamepad1.left_stick_y;
        right_stick_x = gamepad1.right_stick_x;

        NavXTemp = gamepad1.left_stick_button;
        left_bumper = gamepad2.left_bumper;
        right_bumper = gamepad2.right_bumper;
        left_trigger = gamepad2.left_trigger;
        right_trigger = gamepad1.right_trigger;
        a_button = gamepad2.a;
        b_button = gamepad2.b;
        x_button = gamepad2.x;
        y_button = gamepad2.y;
        b_button1 = gamepad1.b;
        a_button1 = gamepad1.a;
        y_button1 = gamepad1.y;
        x_button1 = gamepad1.x;
        LX = gamepad2.left_stick_y;
        RX = gamepad2.right_stick_y;
        dup = gamepad2.dpad_up;
        ddown = gamepad2.dpad_down;
        dleft = gamepad2.dpad_left;
        dright = gamepad2.dpad_right;
        left_bump1 = gamepad1.left_bumper;
        right_bump1 = gamepad1.right_bumper;
        d_down1 = gamepad1.dpad_down;
        d_up1 = gamepad1.dpad_up;
        d_left1 = gamepad1.dpad_left;
        d_right1 = gamepad1.dpad_right;
        stick_press = gamepad2.right_stick_button;
        stick_press1 = gamepad2.left_stick_button;

        robot.grabber.setPower(1);
        robot.dumper.setPower(0.4);


        //slight adjustments for driver
        if(d_down1 == true) {
            left_stick_y = 0.4;
        }
        if(d_up1 == true) {
            left_stick_y = -0.4;
        }
        if(d_left1 == true) {
            left_stick_x = -0.4;
        }
        if(d_right1 == true) {
            left_stick_x = 0.4;
        }

        //changes front of robot for driver using a,b,x,y
        if(a_button1 == true || aPressed == true) {
            front = left_stick_y * -1;
            side = left_stick_x * -1;

            aPressed = true;
            bPressed = false;
            xPressed = false;
            yPressed = false;
        }
        if(b_button1 == true || bPressed == true) {
            front = left_stick_x;
            side = left_stick_y*-1;

            aPressed = false;
            bPressed = true;
            xPressed = false;
            yPressed = false;
        }
        if(x_button1 == true || xPressed == true) {
            front = left_stick_x*-1;
            side = left_stick_y;

            aPressed = false;
            bPressed = false;
            xPressed = true;
            yPressed = false;
        }
        if(y_button1 == true || yPressed == true) {
            front = left_stick_y;
            side = left_stick_x;

            aPressed = false;
            bPressed = false;
            xPressed = false;
            yPressed = true;
        }

        robot.onmiDrive(side, front, right_stick_x);

        //grabber position
        if (left_bumper == true) {
            robot.grabber.setTargetPosition(1500);

        }
        else if(left_trigger > 0.2) {
            robot.grabber.setTargetPosition(1000);
        }
        else {
            robot.grabber.setTargetPosition(0);
        }
        //wheelie controlls
        if(left_bump1 == true) {
            robot.wheelie.setPower(-1.0);
        }
        else if(right_bump1){
            robot.wheelie.setPower(1.0);
        }
        else {
            robot.wheelie.setPower(0.0);
        }

        //dumper controlls
        if (right_bumper == true) {
            robot.dumper.setTargetPosition(480);
        }
        else {
            robot.dumper.setTargetPosition(0);
        }

        //claw controlls
        //closes claws
        if (x_button == true) {
            robot.claw1.setPosition(0.7);
            robot.claw2.setPosition(0.3);
        }
        //all the way open
        else if(y_button == true) {
            robot.claw1.setPosition(0.3);
            robot.claw2.setPosition(0.7);
        }
        //part way open when not pressing a button
        else {
            robot.claw1.setPosition(0.55);
            robot.claw2.setPosition(0.4);
        }

        if(b_button == true) {
            int pos = robot.grabber.getCurrentPosition();
            robot.grabber.setTargetPosition(pos-5);
            robot.grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //reel controlls
        /*oif (dup == true) {
            robot.reel.setPower(.75);
        }
        else if (ddown == true) {
            robot.reel.setPower(-.2);
        }
        else {
            robot.reel.setPower(0);
        }
        //slide controlls
        if(dleft == true) {
            robot.slide.setPower(.5);
        }
        else  if (dright == true) {
            robot.slide.setPower(-.5);
        }
        else {
            robot.slide.setPower(-.02);

        }


        if((stick_press==true || stick_press1 == true) && closed == false && (runtime.seconds() > .5)){
            robot.clamp.setPosition(0.55);
            closed = true;
            runtime.reset();
        }
        else if((stick_press==true || stick_press1==true)&& closed==true && (runtime.seconds() > 0.5)){
            robot.clamp.setPosition(0.1);
            closed = false;
            runtime.reset();
        }
        //if (a_button == true) {
            //robot.wrist.setPosition((robot.wrist.getPosition()-0.1));
        //}
        if(a_button==true && wrist_num >= 0 ) {
            wrist_num = wrist_num -  0.01;
            robot.wrist.setPosition(wrist_num);
        }
        else    if (b_button==true && wrist_num <=2) {
            wrist_num = wrist_num+0.01;
            robot.wrist.setPosition(wrist_num);
        }
        else {
            robot.wrist.setPosition(wrist_num);
        }*/

        /*if (b_button2 == true) {
            robot.grabber.setPower(-0.1);

        }
        else {
            robot.grabber.setPower(0);
        }*/


        //change
        //robot.JKnock.setPosition(0.9);

        //robot.relicLifter(dup,ddown,dleft,dright);

        //robot.grabber.setPosition(0.0);
        // Send telemetry message to signify robot running;

        telemetry.addLine("Controller Telemetry:");
        telemetry.addData("Left Bumper: ", left_bumper);
        telemetry.addData("Right Bumper: ", right_bumper );
        telemetry.addData("Left Trigger: ", left_trigger);
        telemetry.addData("Right Trigger: ", right_trigger);
        telemetry.addData("A Button: ",a_button);
        telemetry.addData("B Button: ",b_button);
        telemetry.addData("X Button: ",x_button);
        telemetry.addData("Y Button: ", y_button);
        telemetry.addData("2nd Left Trigger",LX);
        telemetry.addData("2nd Right Trigger",RX);
        telemetry.addData("Wrist Position: ",wrist_num);
        //telemetry.addData("Ultra front", robot.ultra_front.getDistance(DistanceUnit.CM));
        telemetry.addData("Ultra back", robot.ultra_back.getDistance(DistanceUnit.CM));
        telemetry.addData("Ultra left", robot.ultra_left.getDistance(DistanceUnit.CM));
        telemetry.addData("Ultra right", robot.ultra_right.getDistance(DistanceUnit.CM));
        telemetry.addLine("What is my name?: Spitz");

        if(NavXTemp == true)
            robot.navx_device.zeroYaw();

        telemetry.addData("NavX compassHeading", robot.navx_device.getCompassHeading());
        telemetry.addData("NavX z", robot.navx_device.getRawGyroZ());
        telemetry.addData("NavX y", robot.navx_device.getRawGyroY());
        telemetry.addData("NavX updating?",robot.yawPIDController.isNewUpdateAvailable(new navXPIDController.PIDResult()));
        telemetry.addData("NavX updating?2",robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult));

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.navx_device.close();
    }

}