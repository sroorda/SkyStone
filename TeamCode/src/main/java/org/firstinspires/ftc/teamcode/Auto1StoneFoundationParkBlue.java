/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="1StoneFoundationParkBlue", group="Linear Opmode")

public class Auto1StoneFoundationParkBlue extends LinearOpMode {
    public final static double SPEED = 0.75;
    public void runOpMode() {
        telemetry.addData("change number", 5);
        telemetry.addData("Status", "Initialized v7");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);
        du.moveIntake(DriveUtility.CLAW_OPEN);
        du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);

        waitForStart();

        if (opModeIsActive()) {

            // Move forward to line up with first block
            du.moveWithEncoder(74, .3);

            // Drop the claw to move the block
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(500);
            du.log("Sleep","");

            // Move backwards
            du.moveWithEncoder(-22, SPEED);
            //sleep(300);

            // Move linear slide to prevent stone from slipping
            du.moveLinearSlideWithEncoders(3);
            //sleep(5000);

            // Strafe towards foundation
            du.strafeLeftDistance(195, 0.6);
            sleep(300);

            // Raise linear slide
            du.moveLinearSlideWithEncoders(10);
            sleep(300);

            // Move forward towards foundation
            du.moveWithEncoder(37, SPEED);
            //sleep(300);

            // Lower foundation claws to grab foundation
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_CLOSE);
            sleep(500);

            // Drop stone in foundation
            du.moveIntake(DriveUtility.CLAW_OPEN);
            sleep(500);

            //rotate right slowly a little
            du.rotateLeft(500, 0.3);

            //move backwards to the wall
            du.moveWithEncoder(-75, SPEED);

            //rotate right to rotate foundation
            du.rotateLeft(1300, 0.3);
            sleep(300);

            //move forward and push foundation against wall
            du.moveWithEncoder(100,SPEED);
            //sleep(300);
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);
            sleep(300);
            du.log("Strafe Right Begin", "");
            du.strafeRightDistance(4, SPEED);
            du.log("Strafe Right End", "");
            du.moveWithEncoder(-95, SPEED);
            du.log("Start Linear Slide", "");
            du.moveLinearSlideWithEncoders(-13);
            du.log("End Linear Slide", "");
            sleep(300);
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(300);


            telemetry.update();

        }
    }




}
