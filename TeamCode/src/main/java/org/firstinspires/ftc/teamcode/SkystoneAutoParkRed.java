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


@Autonomous(name="SkystoneAutoParkRed", group="Linear Opmode")

public class SkystoneAutoParkRed extends LinearOpMode {
    public final static double SPEED = 0.75;
    public void runOpMode() {
        telemetry.addData("Status", "Initialized v6");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);
        du.moveIntake(DriveUtility.CLAW_OPEN);
        du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);

        waitForStart();
        du.moveIntake(0.5);

        if (opModeIsActive()) {

            du.moveWithEncoder(28, 1, true);
            sleep(200);
            int position = du.tensorFlow();
            if (position == 1){
                // Move forward to line up with the block
                du.log("BEFORE", "Move to Stone");
                du.moveWithEncoder(51, 1, true);
            }
            else if (position == 2){
                // Strafe to the second block
                du.log("BEFORE STRAFE LEFT", "Move to stone");
                du.strafeLeftDistance(18,1,true);
                // Move forward to line up with the block
                du.log("BEFORE", "Grab stone");
                du.moveWithEncoder(51, 1, true);
            }
            else{
                // Strafe to the second block
                du.log("BEFORE STRAFE LEFT", "Move to stone");
                du.strafeLeftDistance(44,1,true);
                // Move forward to line up with the block
                du.log("BEFORE", "Grab stone");
                du.moveWithEncoder(51, 1, true);
            }
            // Drop the claw to move the block
            du.moveIntake(DriveUtility.CLAW_CLOSE);

            // Move backwards
            du.moveWithEncoder(-18, SPEED);

            sleep(10000);
            telemetry.update();

        }
    }




}
