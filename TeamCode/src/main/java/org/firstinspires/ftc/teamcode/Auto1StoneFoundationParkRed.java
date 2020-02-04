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




@Autonomous(name="1StoneFoundationParkRed", group="Linear Opmode")

public class Auto1StoneFoundationParkRed extends LinearOpMode {
    public final static double SPEED = 0.75;
    public void runOpMode() {
        telemetry.addData("Status", "Initialized v6");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);
        du.moveIntake(DriveUtility.CLAW_OPEN);
        du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);

        waitForStart();

        if (opModeIsActive()) {

            // Move forward to line up with first block
            du.log("BEFORE", "Move to Stone");
            du.moveWithEncoder(74, .3);

            // Drop the claw to move the block
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(500);

            // Move backwards
            du.log("BEFORE", "Move Backwards");
            du.moveWithEncoder(-22, SPEED);
            sleep(300);

            // Move linear slide to prevent stone from slipping
            du.moveLinearSlideWithEncoders(3);
            sleep(100);

            // Strafe towards foundation
            du.log("BEFORE", "Strafe towards foundation");
            du.strafeRightDistance(170, 0.8);
            sleep(300);

            // Raise linear slide
            du.moveLinearSlideWithEncoders(10);
            sleep(300);

            // Move forward towards foundation
            du.log("BEFORE", "Move forward towards foundation");
            du.moveWithEncoder(40, 0.5);
            //sleep(300);

            // Lower foundation claws to grab foundation
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_CLOSE);
            sleep(500);

            // Drop stone in foundation
            du.moveIntake(DriveUtility.CLAW_OPEN);
            sleep(500);

            //rotate right slowly a little
            du.log("BEFORE", "Rotate slowly a little");
            du.rotateRight(500, 0.3);

            //move backwards to the wall
            du.log("BEFORE", "Move backwards to wall");
            du.moveWithEncoder(-75, SPEED);

            //rotate right to rotate foundation
            du.log("BEFORE", "Rotate right to rotate foundation");
            du.rotateRight(1500, 0.4);
            sleep(300);

            //move forward and push foundation against wall
            du.log("BEFORE", "Move forward and push foundation against wall");
            du.moveWithEncoder(100,SPEED);

            //open the foundation claws so we let go of the foundation
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);
            sleep(300);

            //move backwards half way
            du.moveWithEncoder(-47.5, SPEED);
            du.log("Start Linear Slide", "");

            //strafe left to park in the spot closest to the middle bridge
            du.strafeLeftDistance(48, SPEED, true);

            //back up the rest of the way
            du.moveWithEncoder(-47.5, SPEED);

            //reset all of our mechanisms
            du.moveLinearSlideWithEncoders(-13);
            du.log("End Linear Slide", "");
            sleep(300);
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(300);
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_CLOSE);

sleep(3000);

            telemetry.update();

        }
    }




}
