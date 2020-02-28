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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="CoachWTestAuto", group="Linear Opmode")

public class CoachWrightAutoTests extends LinearOpMode {
    public final static double SPEED = 0.75;
    public void runOpMode() {
        telemetry.addData("Status", "Initialized v14");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);
        du.moveIntake(DriveUtility.CLAW_OPEN);
        du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);


        waitForStart();
        du.moveIntake(0.5);

        if (opModeIsActive()) {

            //du.angleCorrectIMU(-90);
            //du.rotateTest(80);
            double starTime = getRuntime();
            //
            // du.rotateViaIMUToAngle(90);
            //du.moveWithEncoder(200,1, false);  // POSITION foundation delivery 1
            DistanceSensor leftSensor;
            leftSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
            double leftSensorDist = leftSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("L range", String.format("%.01f cm", leftSensorDist));

            telemetry.addData("Rotate time", (getRuntime()-starTime));
            telemetry.update();
            /*double foundationDelivery1 = 0;
            double quarry = 0;
            double foundationDelivery2 = 0;


            du.moveWithEncoder(28, 1, false);
            sleep(200);
            int position = du.tensorFlow();

            //systone recognition code
            if (position == 1){
                // Move forward to line up with the block
                du.log("BEFORE", "Move to Stone");
                du.moveWithEncoder(51, 1, true);

                foundationDelivery1 = 170;
                quarry = -230;
                foundationDelivery2 = 230;
            }
            else if (position == 2){
                // Strafe to the second block
                du.log("BEFORE STRAFE LEFT", "Move to stone");
                du.strafeLeftDistance(22,0.3,true);
                du.log("BEFORE ANGLE CORRECT", "Angle correct before grab");
                //du.angleCorrect();
                du.angleCorrectIMU(0);
                // Move forward to line up with the block
                du.log("BEFORE", "Grab stone");
                du.moveWithEncoder(40, 1, false);

                foundationDelivery1 = 40;//200;
                quarry = -260;
                foundationDelivery2 = 233;
            }
            else{
                // Strafe to the second block
                du.log("BEFORE STRAFE LEFT", "Move to stone");
                du.strafeLeftDistance(44,1,true);

                // Move forward to line up with the block
                du.log("BEFORE", "Grab stone");
                du.moveWithEncoder(51, 1, true);

                foundationDelivery1 = 235;
                quarry = -265;
                foundationDelivery2 = 265;
            }



            // Drop the claw to move the block
            du.log("BEFORE WE MOVE INTAKE", "");
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(300);

            //move linear slide up
            du.moveLinearSlideWithRunUsingEncoders(2);

            // Move backwards
            du.log("BEFORE", "Move backwards after we grab block");
            du.moveWithEncoder(-18,SPEED); //coach

            //strafe towards foundation
            du.log("BEFORE", "Strafe towards foundation");
            du.rotate(-67, .5); //coach
            du.log("BEFORE", "angle correct imu 2");
            du.angleCorrectIMU(90);
            du.moveWithEncoder(foundationDelivery1,1, false);  // POSITION foundation delivery 1
     /*
            du.rotate(67,0.5);

            // Raise linear slide
            du.moveLinearSlideWithRunUsingEncoders(15); //coach

            // Move forward towards foundation
            du.log("BEFORE", "Move forward towards foundation");
            du.moveWithEncoder(20, 0.5); //coach

            // Drop stone in foundation
            du.moveIntake(DriveUtility.CLAW_OPEN);

            //move backwards after dropping stone in foundation
            du.moveWithEncoder(-22, 1, false);

            //lower linear slide
            du.moveLinearSlideWithRunUsingEncodersDown(13);

            //turn and move forward to second stone
            du.rotate(-67, 0.5);
            du.log("BEFORE", "ANGLE CORRECT");
            du.angleCorrect();
            du.log("AFTER", "ANGLE CORRECT");

            du.moveIntake(DriveUtility.CLAW_CLOSE);
            du.moveWithEncoder(quarry,1,false); // POSITION quarry

            //rotate to face the 2nd skystone
            du.moveIntake(DriveUtility.CLAW_OPEN);
            du.rotate(67, 0.5);
/*
            //move forward to grab 2nd block
            du.moveWithEncoder(25,1,false);

            //grab the 2nd block
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(300);
            du.moveLinearSlideWithRunUsingEncoders(2);

            // back up
            du.moveWithEncoder(-20,1,false);

            //move forward to get to the foundation
            du.rotate(-80, .5);
            du.moveWithEncoder(foundationDelivery2,1, false); // POSITION foundation delivery 2
            du.rotate(67, .5);

            //move forward to grab the foundation
            du.moveLinearSlideWithRunUsingEncoders(13);
            du.moveWithEncoder(15,1,false);
            du.moveIntake(DriveUtility.CLAW_OPEN);
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_CLOSE);
/*
            //move foundation into building zone
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
            du.strafeLeftDistance(40, SPEED, true);

            //back up the rest of the way
            du.moveWithEncoder(-47.5, SPEED);

            //reset all of our mechanisms
            du.moveLinearSlideWithEncoders(-13);
            du.log("End Linear Slide", "");
            sleep(300);
            du.moveIntake(DriveUtility.CLAW_CLOSE);
            sleep(300);
            du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_CLOSE);

*/
            sleep(3000);
            telemetry.update();

        }
    }




}