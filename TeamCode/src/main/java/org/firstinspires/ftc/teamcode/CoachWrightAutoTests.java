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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



@Autonomous(name="CoachWTests", group="Linear Opmode")
@Disabled
public class CoachWrightAutoTests extends LinearOpMode {
    public final static double SPEED = 0.75;

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    public static double TICKS_PER_CM = 17.1;

    public void runOpMode() {
        telemetry.addData("change number", 2);
        telemetry.addData("Status", "Initialized v1");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);
        du.moveIntake(DriveUtility.CLAW_OPEN);
        du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);

        for(DcMotor m : du.motorList ) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        sleep(1000);


        waitForStart();
        resetStartTime();

        double an = 0;

        int dir = 1;
        double pf = 0;
        double pfTarget = .9;
        double pfRampStep = 0.05;
        double rampTime = 1;
        double numRampSteps = pfTarget/pfRampStep;
        double rampCheckTime = rampTime/numRampSteps;
        double lastRampCheck = 0;
        double corBack = 1;
        double corFront = 1;
        double corVal = 0.001;//0.00001;
        double angleFuzz = 0.0000001;

        double distToTravelInFt = 8;
        double distToTravelInCM = distToTravelInFt*30.48;
        double distToTravelInTicks = distToTravelInCM*TICKS_PER_CM;
        double currentDistInTicks = 0;

        double distToRampDownInFt = 4;
        double distToRampDownInCM = distToRampDownInFt*30.48;
        double distToRampDownInTicks = distToRampDownInCM*TICKS_PER_CM;
        boolean rampDown = false;


        double accelFactor = 1;

        int rampingYesNo = 1;

        boolean movingToTarget = true;
        double rotatAngleTarget = 2;

        while (opModeIsActive() && getRuntime() < 10) {
            int br = du.motorList.get(0).getCurrentPosition();
            int fr = du.motorList.get(1).getCurrentPosition();
            int bl = du.motorList.get(2).getCurrentPosition();
            int fl = du.motorList.get(3).getCurrentPosition();
            if(movingToTarget) {
                currentDistInTicks = (Math.abs(fr) + Math.abs(br) + Math.abs(bl) + Math.abs(fl)) / 4;
                if (currentDistInTicks > distToTravelInTicks) {
                    du.motorList.get(0).setPower(0);  // br
                    du.motorList.get(1).setPower(0);  // fr
                    du.motorList.get(2).setPower(0);  // bl
                    du.motorList.get(3).setPower(0);  // fl
                    movingToTarget = false;
                } else {
                    du.motorList.get(0).setPower(-1 * -dir * pf * corBack * accelFactor);  // br
                    du.motorList.get(1).setPower(1 * -dir * pf * corFront);  // fr
                    du.motorList.get(2).setPower(1 * dir * pf * corBack * accelFactor);  // bl
                    du.motorList.get(3).setPower(-1 * dir * pf * corFront);  // fl
                }


                if (currentDistInTicks > distToTravelInTicks - distToRampDownInTicks) {
                    rampDown = true;
                    pf = pf - 0.01;
                    if (pf < 0) {
                        pf = 0;
                    }
                }

                if (pf < pfTarget && (getRuntime() - lastRampCheck) > rampCheckTime && !rampDown) {
                    lastRampCheck = getRuntime();
                    pf = pf + pfRampStep;
                    rampingYesNo = 1;
                    accelFactor = 1.08;
                } else if (pf < pfTarget) {
                    rampingYesNo = 1;
                } else {
                    rampingYesNo = 0;
                    accelFactor = 1.08;
                }
            }
            else
            {
                if(an > rotatAngleTarget)
                {
                    du.motorList.get(0).setPower(0.05);  // br
                    du.motorList.get(1).setPower(0.05);  // fr
                    du.motorList.get(2).setPower(0.05);  // bl
                    du.motorList.get(3).setPower(0.05);  // fl
                }
                else if (an < -rotatAngleTarget)
                {
                    du.motorList.get(0).setPower(-0.05);  // br
                    du.motorList.get(1).setPower(-0.05);  // fr
                    du.motorList.get(2).setPower(-0.05);  // bl
                    du.motorList.get(3).setPower(-0.05);  // fl
                }
                else
                {
                    du.motorList.get(0).setPower(0);  // br
                    du.motorList.get(1).setPower(0);  // fr
                    du.motorList.get(2).setPower(0);  // bl
                    du.motorList.get(3).setPower(0);  // fl
                }

            }



            an = getAngle();
            //for(DcMotor m : du.motorList ) {
            //   a = m.getCurrentPosition();

            //}
            telemetry.addData("rotating",!movingToTarget);
            telemetry.addData("time: ",getRuntime());
            //telemetry.addData("br: ", br);
            //telemetry.addData("fr: ", fr);
            //telemetry.addData("bl: ", bl);
            //telemetry.addData("fl: ", fl);
            telemetry.addData("angle: ", an);
            telemetry.addData("ramping: ", rampDown);
            //telemetry.addData("corFront: ",corFront);
            //telemetry.addData("corBack: ",corBack);
            telemetry.addData("currentTicks: ",Math.round(currentDistInTicks));
            telemetry.addData(" rampingDown: ",Math.round(distToTravelInTicks-distToRampDownInTicks));
            telemetry.addData(" targetTicks: ",Math.round(distToTravelInTicks));
            telemetry.update();
            idle();

            if(an>angleFuzz) {
                corBack = corBack * (1+corVal);
                corFront = corFront * (1-corVal);
            }
            if(an < -angleFuzz) {
                corBack = corBack*(1-corVal);
                corFront = corFront * (1+corVal);
            }

        }
        du.motorList.get(0).setPower(0);  // br
        du.motorList.get(1).setPower(0);  // fr
        du.motorList.get(2).setPower(0);  // bl
        du.motorList.get(3).setPower(0);  // fl
        sleep(5000);
    }


    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

}
