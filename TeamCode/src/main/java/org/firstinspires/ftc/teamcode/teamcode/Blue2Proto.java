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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Omnibot: Blue2Proto", group="Omnibot")
//@Disabled
public class Blue2Proto extends LinearOpMode {

    HardwareOmniRobot  robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap);
        robot.navx_device.zeroYaw();
        //robot.yawPIDResult = new navXPIDController.PIDResult();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //Vuforia Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int choosen = robot.Vuforia(cameraMonitorViewId, "blue");
        telemetry.addData("VuMark", "%s visible", choosen);
        telemetry.update();

        robot.JewelKnock("blue");
        robot.DriveFor(0.3,0.0,0.0,0.0);
        robot.wheelie.setPower(1.0);
        robot.DriveFor(1.2,1.0,0.0,0.0);
        robot.wheelie.setPower(0.0);
        robot.DriveFor(0.3,0.0,0.0,0.0);



        //turn 180
        robot.NavXInit(180);
        runtime.reset();

        while(robot.navx_device.isMoving() == true && runtime.seconds() < 1.5) {
            telemetry.addData("NavX moving1", robot.navx_device.isMoving());
            telemetry.addData("NavX Updating1?", robot.yawPIDController.isNewUpdateAvailable(new navXPIDController.PIDResult()));
            telemetry.addData("NavX Updating2?", robot.yawPIDController.isNewUpdateAvailable(robot.yawPIDResult));
            telemetry.update();
            robot.NavX(0.0,0.0);
        }

        int target;
        switch(choosen) {
            case(3):
                target = 85;
                break;
            case(2):
                target = 67;
                break;
            case(1):
                target = 50;
                break;
            default:
                target = 50;
                telemetry.addData("default target", choosen);
                telemetry.update();
                break;
        }

        robot.grabber.setTargetPosition(0);
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.6);

        boolean inPlace = false;
        boolean dis = false;
        while(dis == false && runtime.seconds() < 20) {
            double distanceRight = robot.ultra_right.getDistance(DistanceUnit.CM);

            telemetry.addData("Back", distanceRight);
            telemetry.update();

            if(distanceRight >= target-1 && distanceRight <= target+1) {
                robot.NavX(0.0,0.0);
                inPlace = true;
                dis = true;
            }
            else if(distanceRight < target-1) {
                robot.NavX(0.0,0.4);
            }
            else {
                robot.NavX(0.0,-0.4);
            }
        }
        if(inPlace == true) {
            dis = false;
            while(dis == false && runtime.seconds() < 25) {
                double distanceBack = robot.ultra_back.getDistance(DistanceUnit.CM);
                telemetry.addData("Left", distanceBack);
                telemetry.update();

                if(distanceBack >= 17-1 && distanceBack <= 17+1) {
                    robot.NavX(0.0,0.0);
                    dis = true;
                }
                else if(distanceBack < 17-1) {
                    robot.NavX(-0.4,0.0);
                }
                else {
                    robot.NavX(0.4,0.0);
                }
            }
            while(robot.dumper.getCurrentPosition() <= 480){robot.dumper.setTargetPosition(480);}
            while(robot.dumper.getCurrentPosition() != 0){robot.dumper.setTargetPosition(0);}
        }
        robot.navx_device.close();
    }
}