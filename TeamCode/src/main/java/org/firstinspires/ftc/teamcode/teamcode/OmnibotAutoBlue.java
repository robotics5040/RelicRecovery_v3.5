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

@Autonomous(name="Omnibot: AutotestBlue", group="Omnibot")
//@Disabled
public class OmnibotAutoBlue extends AutoPull {

    HardwareOmniRobot  robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap, false);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //Vuforia Stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int choosen = Vuforia(cameraMonitorViewId, "blue");
        telemetry.addData("VuMark", "%s visible", choosen);
        telemetry.update();

        JewelKnock(robot,"blue");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        DriveFor(robot,1.2,1.0,0.0,0.0);
        DriveFor(robot,1.0,0.0,0.0,0.0);

        //setting distance to choosen column
        int target;
        boolean inPlace = false;
        if(choosen == 3) {
            target = 57;
        }
        if(choosen == 2) {
            target = 39;
        }
        else {
            target = 24;
        }

        //move distance from column wall     19 with columns?
         boolean dis = false;
        while(dis == false) {
            double distanceBack = robot.ultra_back.getDistance(DistanceUnit.CM);

            telemetry.addData("Back", distanceBack);
            telemetry.update();

            if(distanceBack >= 17 && distanceBack <= 17) {
                dis = true;
            }
            else if(distanceBack <= 17) {
            }
            else if(distanceBack >= 17) {
            }
        }

        //moving to correct column
        while(inPlace == false) {
            double distanceLeft = robot.ultra_right.getDistance(DistanceUnit.CM);

            telemetry.addData("Left", distanceLeft);
            telemetry.update();

            if(distanceLeft==target) {
                while(robot.dumper.getCurrentPosition() <= 175){robot.dumper.setTargetPosition(180);}
                while(robot.dumper.getCurrentPosition() != 0){robot.dumper.setTargetPosition(0);}
                inPlace = true;
            }
            else if(distanceLeft > target) {
            }
            else if(distanceLeft < target && distanceLeft > target-2) {
            }
            else {
            }
        }



        robot.grabber.setTargetPosition(0);
        DriveFor(robot,1.0,0.0,0.0,0.0);
    }
}