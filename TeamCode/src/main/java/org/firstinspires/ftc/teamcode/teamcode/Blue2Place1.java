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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

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

@Autonomous(name="Omnibot: Blue2Place1", group="Omnibot")
//@Disabled
public class Blue2Place1 extends AutoPull {

    HardwareOmniRobot robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime runtime2 = new ElapsedTime();

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);

        robot.grabber.setPower(0.75);
        robot.grabber.setTargetPosition(robot.GRABBER_AUTOPOS);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AUBrQCz/////AAAAGXg5njs2FEpBgEGX/o6QppZq8c+tG+wbAB+cjpPcC5bwtGmv+kD1lqGbNrlHctdvrdmTJ9Fm1OseZYM15VBaiF++ICnjCSY/IHPhjGW9TXDMAOv/Pdz/T5H86PduPVVKvdGiQ/gpE8v6HePezWRRWG6CTA21itPZfj0xDuHdqrAGGiIQXcUbCTfRAkY7HwwRfQOM1aDhmeAaOvkPPCnaA228iposAByBHmA2rkx4/SmTtN82rtOoRn3/I1PA9RxMiWHWlU67yMQW4ExpTe2eRtq7fPGCCjFeXqOl57au/rZySASURemt7pwbprumwoyqYLgK9eJ6hC2UqkJO5GFzTi3XiDNOYcaFOkP71P5NE/BB    ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

       /* while (robot.gyro.isCalibrating() && robot.gyro2.isCalibrating()){
            telemetry.addLine("Calibrating gyro");
            telemetry.update();
        }
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("heading", robot.gyro.getHeading());
            telemetry.update();
            idle();
        }

        int startG = robot.gyro.getHeading();*/

        runtime.reset();
        runtime2.reset();

        int choosen = Vuforia(cameraMonitorViewId, "blue", vuforia);
        int target = 0;

        switch (choosen) {
            case (1):
                target = 22;
                break;
            case (2):
                target = 29;
                break;
            case (3):
                target = 36;
                break;
            default:
                target = 29;
                break;
        }

        telemetry.addData("VuMark", "%s visible", choosen);
        telemetry.update();

        robot.claw1.setPosition(0.55);
        robot.claw2.setPosition(0.45);

        JewelKnock(robot,"blue");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        if(robot.jknock.getPosition() != robot.JKUP) {robot.jknock.setPosition(robot.JKUP);}
        robot.wheelie.setPower(1);
        DriveFor(robot,0.7,1.0,0.0,0.0);
        robot.wheelie.setPower(0);
        DriveFor(robot,0.5,0.45,0.0,0.0);
        DriveFor(robot,0.7,-0.45,0.0,0.0);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        DriveFor(robot,1.0,0.0,0.0,1.0);
        DriveFor(robot,0.3,0,0,0);
        RotateTo(robot,90, -90);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        //line up back
        DriveFor(robot, 0.3,0,-1,0);
        DriveFor(robot,0.6,-1,0,0);
        DriveFor(robot,0.3,0.5,0,0);
        DriveFor(robot,0.3,0,0,0);

        boolean dis = false;

        DriveFor(robot,0.3,0.0,0.0,0.0);

        telemetry.addLine("Lineup 1 Complete");
        telemetry.update();

        boolean dis2 = false;
        int count = 0;
        runtime.reset();
        while (dis2 == false && runtime2.seconds() < 26 && opModeIsActive() && dis == true) {
            double distanceRight = ((robot.ultra_right.getVoltage() / 5) * 512) + 2.5;// robot.ultra_right.getDistance(DistanceUnit.CM);
            telemetry.addData("Right", distanceRight);
            telemetry.update();

            if (distanceRight > target+1) {
                onmiDrive(robot, -0.45, 0.0, 0.0);
            }
            else if (distanceRight < target-1) {
                onmiDrive(robot,0.45,0.0,0.0);
            }
            else {
                onmiDrive(robot,0.0, 0.0, 0.0);
                if(count >= 2) {
                    dis2 = true;
                }
                else {
                    count ++;
                    DriveFor(robot,0.3,0,0,0);
                    //RotateTo(robot,90, -90);
                    //DriveFor(robot,0.3,0,0,0);
                    runtime.reset();
                }
            }
            if(runtime.seconds() > 1.0 && choosen != 1) {
                DriveFor(robot,0.3,0,0,0);
                //RotateTo(robot,90, -90);
                //DriveFor(robot,0.3,0,0,0);
                runtime.reset();
            }
        }

        telemetry.addLine("Lineup 2 Complete");
        telemetry.update();

        robot.dumper.setPower(0.4);
        onmiDrive(robot,0.0, 0.0, 0.0);
        runtime.reset();
        while (robot.dumper.getCurrentPosition() <= 470 && opModeIsActive() && runtime2.seconds() < 28 && runtime.seconds() < 1.5) {
            robot.dumper.setTargetPosition(480);
        }

        DriveFor(robot,0.5,0.0,0.0,0.0);
        DriveFor(robot,0.5, 0.5, 0.0, 0.0);

        while (robot.dumper.getCurrentPosition() >= 5 && opModeIsActive()) {
            robot.dumper.setTargetPosition(0);
        }

        if(runtime2.seconds() < 29) {
            DriveFor(robot, 1.0, -0.8, 0.0, 0.0);
            DriveFor(robot, 0.5, 0.45, 0.0, 0.0);
        }
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.7);
        DriveFor(robot,1.0, 0.0, 0.0, 0.0);

    }
}
