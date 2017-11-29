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
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name="Omnibot: Blue1Simple", group="Omnibot")
//@Disabled
public class Blue1Simple extends AutoPull {

    HardwareOmniRobot robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, false);

        //sets position of the grabber for auto - stays up
        robot.grabber.setPower(0.75);
        robot.grabber.setTargetPosition(robot.GRABBER_AUTOPOS);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        waitForStart();
        runtime.reset();
        //sets claws in so they do not break when turning
        robot.claw1.setPosition(0.7);
        robot.claw2.setPosition(0.3);

        //runs from knocking off jewel to driving off platform.
        JewelKnock(robot,"blue");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        if(robot.jknock.getPosition() != robot.JKUP) {robot.jknock.setPosition(robot.JKUP);}
        robot.wheelie.setPower(1);
        DriveFor(robot,1.4,1.0,0.0,0.0);
        robot.wheelie.setPower(0);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        //goes distance from wall to be in park zone
        boolean dis = false;
        while(opModeIsActive() && dis == false && runtime.seconds() < 26) {
            double distanceLeft = robot.ultra_left.getDistance(DistanceUnit.CM);

            telemetry.addData("Left", distanceLeft);
            telemetry.update();

            if(distanceLeft >= 16 && distanceLeft <= 18) {
                telemetry.addData("Done", distanceLeft);
                telemetry.update();
                RobotLog.ii("5040MSG","Done",distanceLeft);

                robot.onmiDrive(0.0,0.0,0.0);
                dis = true;
            }
            else if(distanceLeft < 16) {
                telemetry.addData("Towards", distanceLeft);
                telemetry.update();
                robot.onmiDrive(0.4,0.0,0.0);
            }
            else {
                telemetry.addData("Away", distanceLeft);
                telemetry.update();
                robot.onmiDrive(-0.4,0.0,0.0);
            }
        }
    }
}