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

/**
 * Autonomous sample of moving with time vs moving with encoders
 *
 * @author Sylvianne J Rodgers
 */

@Autonomous(name="Autonomous Test with Encoders", group="Linear Opmode")
public class AutoTestWithEncoder extends LinearOpMode {

    private DistanceSensor leftSensor;
    private DistanceSensor rightSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized v5");
        telemetry.update();
/*
        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
        double leftSensorDist = -1;
        double rightSensorDist = -1;
        double surfaceAngle = 0;
        double adjacentDist = 24.8; // in cm
        double oppositeDist = 0;
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
*/

        DriveUtility du = new DriveUtility(hardwareMap, telemetry, this);
        //////////////////////////////////////////////////////////////////////////////
        waitForStart();

        double opmodeAngle = 0;
        while (opModeIsActive()) {
            du.setPower();
/*
            du.setMotorSpeeds(1, 0.3, 0, 0, 0);
            sleep(1000);
            du.setMotorSpeeds(1, 0, 0.3, 0, 0);
            sleep(1000);
            du.setMotorSpeeds(1, 0, 0, 0.3, 0);
            sleep(1000);
            du.setMotorSpeeds(1, 0, 0, 0, 0.3);
            sleep(1000);

/*
            opmodeAngle = du.distSensorsAngle();
            telemetry.addData("  angle", String.format("%.01f deg", opmodeAngle));


            leftSensorDist = leftSensor.getDistance(DistanceUnit.CM);
            rightSensorDist = rightSensor.getDistance(DistanceUnit.CM);

            if ((leftSensorDist > 50) || (rightSensorDist > 50)) {

            } else {
                oppositeDist = rightSensorDist - leftSensorDist;
                surfaceAngle = Math.toDegrees(Math.atan(oppositeDist / adjacentDist));

            }

            telemetry.addData("opp", String.format("%.01f cm", oppositeDist));
            telemetry.addData("adj", String.format("%.01f cm", adjacentDist));
            telemetry.addData("L range", String.format("%.01f cm", leftSensorDist));
            telemetry.addData("R range", String.format("%.01f cm", rightSensorDist));
            telemetry.addData("  angle", String.format("%.01f deg", surfaceAngle));
*/
            telemetry.update();
        }
    }
}



