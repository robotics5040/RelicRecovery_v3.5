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

@Autonomous(name="Omnibot: Red2Proto", group="Omnibot")
//@Disabled
public class Red2Proto extends AutoPull {

    HardwareOmniRobot robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap, false);
        //robot.yawPIDResult = new navXPIDController.PIDResult();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Vuforia Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int choosen = Vuforia(cameraMonitorViewId, "red");
        int target = 0;

        switch(choosen) {
            case(1):
                target = 103;
                break;
            case(2):
                target = 118;
                break;
            case(3):
                target = 68;
                break;
            default:
                target = 103;
                break;
        }
        telemetry.addData("VuMark", "%s visible", choosen);
        telemetry.update();

        JewelKnock(robot,"red");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        DriveFor(robot,1.1,-1.0,0.0,0.0);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        DriveFor(robot,1.8,0.0,0.0,0.5);

        robot.grabber.setTargetPosition(0);
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.6);

        boolean dis = false;
        while(dis == false && runtime.seconds() < 26) {
            double distanceBack = robot.ultra_back.getDistance(DistanceUnit.CM);

            telemetry.addData("Back", distanceBack);
            telemetry.update();

            if(distanceBack >= 58 && distanceBack <= 60) {
                robot.onmiDrive(0.0,0.0,0.0);
                dis = true;
            }
            else if(distanceBack < 58) {
                robot.onmiDrive(0.0,-0.4,0.0);
            }
            else {
                robot.onmiDrive(0.0,0.4,0.0);
            }
        }
        //while(runtime.seconds() < 28) {robot.jknock.setPosition(0.59);}
    }
}