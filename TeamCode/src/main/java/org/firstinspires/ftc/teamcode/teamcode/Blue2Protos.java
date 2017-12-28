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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

@Autonomous(name="Omnibot: Blue2Protos", group="Omnibot")
//@Disabled
public class Blue2Protos extends AutoPull {

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

        while (!(isStarted() || isStopRequested())) {
            float heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Display the light level while we are waiting to start
            telemetry.addData("HEADING",heading);
            telemetry.update();
            idle();
        }

        runtime.reset();
        runtime2.reset();

        int choosen = Vuforia(cameraMonitorViewId, "blue", vuforia);
        int target = 0;

        switch (choosen) {
            case (1):
                target = 50;
                break;
            case (2):
                target = 66;
                break;
            case (3):
                target = 84;
                break;
            default:
                target = 50;
                break;
        }

        telemetry.addData("VuMark", "%s visible", choosen);
        telemetry.update();

        JewelKnock(robot, "blue");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        if(robot.jknock.getPosition() != robot.JKUP) {robot.jknock.setPosition(robot.JKUP);}
        robot.wheelie.setPower(1);
        DriveFor(robot,1.0,1.0,0.0,0.0);
        robot.wheelie.setPower(0);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        telemetry.update();

        //DriveFor(robot,1.0,0.0,0.0,1.0);
        DriveFor(robot,0.3,0,0,0);
        RotateTo180(robot,180);


        robot.claw1.setPosition(0.5);
        robot.claw2.setPosition(0.4);

        boolean dis = false;

        DriveFor(robot,0.5,0.0,0.0,0.0);
        while (dis == false && runtime2.seconds() < 24 && opModeIsActive()) {
            double distanceBack = ((robot.ultra_back.getVoltage() / 5) * 512) + 2.5;//robot.ultra_back.getDistance(DistanceUnit.CM);

            telemetry.addData("Back", distanceBack);
            telemetry.update();

            if (distanceBack == 17) {
                onmiDrive(robot,0.0, 0.0, 0.0);
                dis = true;
            } else if (distanceBack > 17) {
                onmiDrive(robot,0.0, -0.45, 0.0);
            } else {
                onmiDrive(robot,0.0, 0.45, 0.0);
            }
        }

        telemetry.addLine("Lineup 1 Complete");
        telemetry.update();

        boolean dis2 = false;
        int count = 0;
        runtime.reset();
        while (dis2 == false && runtime2.seconds() < 26 && opModeIsActive() && dis == true) {
            double distanceRight = ((robot.ultra_right.getVoltage() / 5) * 512) + 2.5;// robot.ultra_right.getDistance(DistanceUnit.CM);
            telemetry.addData("Right", distanceRight);
            telemetry.update();

            if (distanceRight > 2) { //eliminates the 1.242445621452 crap
                if (distanceRight == target) {
                    onmiDrive(robot,0.0, 0.0, 0.0);
                    if(count >= 1) {
                        dis2 = true;
                    }
                    else {
                        count ++;
                        DriveFor(robot,0.3,0,0,0);
                        RotateTo180(robot,180);
                        DriveFor(robot,0.3,0,0,0);
                        runtime.reset();
                    }

                } else if (distanceRight < target) {
                    onmiDrive(robot,0.45,0.0,0.0);
                    //NavX(0.0, -0.3);
                } else {
                    onmiDrive(robot, -0.45, 0.0, 0.0);
                    //NavX(0.0, 0.3);
                }
                if(runtime.seconds() > 1.0 && choosen != 1) {
                    DriveFor(robot,0.3,0,0,0);
                    RotateTo180(robot,180);
                    DriveFor(robot,0.3,0,0,0);
                    runtime.reset();
                }
            }
            else {
                onmiDrive(robot,0,0,0);
            }
        }

        telemetry.addLine("Lineup 2 Complete");
        telemetry.update();

        robot.dumper.setPower(0.4);
        onmiDrive(robot,0.0, 0.0, 0.0);
        while (robot.dumper.getCurrentPosition() <= 470 && opModeIsActive() && runtime2.seconds() < 28) {
            robot.dumper.setTargetPosition(480);
        }
        while (robot.dumper.getCurrentPosition() >= 5 && opModeIsActive()) {
            robot.dumper.setTargetPosition(0);
        }


        DriveFor(robot,0.5,0.0,0.0,0.0);
        DriveFor(robot,0.5,0.45,0.0,0.0);
        if(runtime2.seconds() < 29) {
            DriveFor(robot, 1.0, -0.8, 0.0, 0.0);
            DriveFor(robot, 0.5, 0.45, 0.0, 0.0);
        }
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.7);
        DriveFor(robot,1.0, 0.0, 0.0, 0.0);

    }
    public void RotateTo180(HardwareOmniRobot robot,int degrees) {
        float heading=robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double speed = 0.5;


        while(heading != degrees  && opModeIsActive()) {
            telemetry.addData("speed",speed);
            telemetry.addData("HEADING",heading);
            telemtery.addData("Status", robot.imu.getSystemStatus());
            telemetry.update();
            heading=robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if(heading < -150) {
                onmiDrive(robot, 0.0, 0.0, -speed);
                if(speed > 0.35) {
                    speed -= 0.01;
                }
                telemetry.addLine("1");
            }
            else if (degrees+2 < heading) {
                onmiDrive(robot, 0.0, 0.0, -speed);
                if(speed > 0.3) {
                    speed -= 0.01;
                }
                telemetry.addLine("2");
            } else if (degrees-2 > heading) {
                onmiDrive(robot, 0.0, 0.0, speed);
                telemetry.addLine("3");
            }
            else {
                onmiDrive(robot,0.0,0.0,0.0);
            }
        }
    }
}
