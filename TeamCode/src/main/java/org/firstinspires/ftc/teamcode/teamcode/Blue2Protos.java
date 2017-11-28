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

    @Override public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);

        robot.grabber.setPower(0.75);
        robot.grabber.setTargetPosition(robot.GRABBER_AUTOPOS);

        while (robot.gyro.isCalibrating()){
            telemetry.addLine("Calibrating gyro");
            telemetry.update();
        }
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("HEADING",robot.gyro.getHeading());
            telemetry.update();
            idle();
        }

        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        waitForStart();
        runtime.reset();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int choosen = Vuforia(cameraMonitorViewId, "blue");
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

        JewelKnock(robot,"blue");
        DriveFor(robot,0.3,0.0,0.0,0.0);
        if(robot.jknock.getPosition() != 0.7) {robot.jknock.setPosition(0.7);}
        robot.wheelie.setPower(1);
        DriveFor(robot,1.0,1.0,0.0,0.0);
        robot.wheelie.setPower(0);
        DriveFor(robot,0.3,0.0,0.0,0.0);

        telemetry.update();

        RotateTo(robot,180);


        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.6);

        boolean dis = false;

        DriveFor(robot,0.5,0.0,0.0,0.0);
        while (dis == false && runtime.seconds() < 20 && opModeIsActive()) {
            double distanceBack = robot.ultra_back.getDistance(DistanceUnit.CM);

            telemetry.addData("Back", distanceBack);
            telemetry.update();

            if (distanceBack == 21) {
                onmiDrive(robot,0.0, 0.0, 0.0);
                dis = true;
            } else if (distanceBack > 21) {
                onmiDrive(robot,0.0, 0.3, 0.0);
            } else {
                onmiDrive(robot,0.0, -0.3, 0.0);
            }
        }

        telemetry.addLine("Lineup 1 Complete");
        telemetry.update();

        boolean dis2 = false;
        while (dis2 == false && runtime.seconds() < 26 && opModeIsActive() && dis == true) {
            double distanceRight = robot.ultra_right.getDistance(DistanceUnit.CM);
            telemetry.addData("Right", distanceRight);
            telemetry.update();

            if (distanceRight > 2) { //eliminates the 1.242445621452 crap
                if (distanceRight == target) {
                    onmiDrive(robot,0.0, 0.0, 0.0);
                    dis2 = true;

                } else if (distanceRight < target) {
                    onmiDrive(robot,-0.3,0.0,0.0);
                    //NavX(0.0, -0.3);
                } else {
                    onmiDrive(robot,0.3,0.0,0.0);
                    //NavX(0.0, 0.3);
                }
            }
        }

        telemetry.addLine("Lineup 2 Complete");
        telemetry.update();

        robot.dumper.setPower(0.4);
        while (robot.dumper.getCurrentPosition() <= 470 && opModeIsActive() && runtime.seconds() < 27) {
            robot.dumper.setTargetPosition(480);
        }
        while (robot.dumper.getCurrentPosition() >= 5 && opModeIsActive()) {
            robot.dumper.setTargetPosition(0);
        }


        DriveFor(robot,0.5,0.0,0.0,0.0);
        DriveFor(robot,0.5, 0.4, 0.0, 0.0);
        DriveFor(robot,1.0, -0.8, 0.0, 0.0);
        DriveFor(robot,0.5, 0.4, 0.0, 0.0);
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.7);
        DriveFor(robot,1.0, 0.0, 0.0, 0.0);

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
            while (opModeIsActive() && choosen == 0 && runtime.seconds() < 3) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if(side == "red") {
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
                    else {
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
                }
            }
        }catch (Exception e){
            choosen = 0;
        }

        return choosen;
    }
}
