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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="CoachWTestDriveStraight", group="Linear Opmode")
@Disabled
public class CoachWrightTestStraight extends LinearOpMode {
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
        boolean movingToTarget = true;
        double rotatAngleTarget = 2;


        double distToTravelInCM = 200;
        int distToTravelInTicks = (int)Math.round(distToTravelInCM*TICKS_PER_CM);
        int dir = 1;
        double pf = .9;
        double ramppf = 0;
        int accelState = 0;

        double lastAccelCheck = 0;
        double accelCheckRate = 0.01;
        double decelDistInCM = 15;
        int maxRunTimeInSec = 30;


        int br = du.motorList.get(0).getCurrentPosition();
        int fr = du.motorList.get(1).getCurrentPosition();
        int bl = du.motorList.get(2).getCurrentPosition();
        int fl = du.motorList.get(3).getCurrentPosition();
        int avgPos = 0;
        while (opModeIsActive() && getRuntime() < maxRunTimeInSec) {
            br = du.motorList.get(0).getCurrentPosition();
            fr = du.motorList.get(1).getCurrentPosition();
            bl = du.motorList.get(2).getCurrentPosition();
            fl = du.motorList.get(3).getCurrentPosition();
            avgPos = (Math.abs(br)+Math.abs(fr)+Math.abs(bl)+Math.abs(fl))/4;
            if(movingToTarget) {
                telemetry.addData("target ticks: ",distToTravelInTicks);
                telemetry.addData("moving to target: ",avgPos);
                du.motorList.get(0).setPower(-dir * pf * ramppf);  // br
                du.motorList.get(1).setPower(-dir * pf * ramppf);  // fr
                du.motorList.get(2).setPower(dir * pf * ramppf);  // bl
                du.motorList.get(3).setPower(dir * pf * ramppf);  // fl

                if(avgPos > distToTravelInTicks){
                    movingToTarget = false;
                }

                if(avgPos > (distToTravelInTicks-decelDistInCM* TICKS_PER_CM)) {
                    accelState = 2;
                }

            }
            else {
                telemetry.addData("target reached",avgPos);
                du.motorList.get(0).setPower(0);  // br
                du.motorList.get(1).setPower(0);  // fr
                du.motorList.get(2).setPower(0);  // bl
                du.motorList.get(3).setPower(0);  // fl

                du.motorList.get(0).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                du.motorList.get(1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                du.motorList.get(2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                du.motorList.get(3).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if((getRuntime() - lastAccelCheck) > accelCheckRate) {
                lastAccelCheck = getRuntime();
                if (accelState == 0) {
                    telemetry.addData("Accelerating", accelState);
                    ramppf = ramppf + 0.05;
                    if (ramppf > 1) {
                        ramppf = 1;
                        accelState = 1;
                    }
                } else if (accelState == 1) {
                    telemetry.addData("At Speed", accelState);
                    ramppf = 1;
                } else if (accelState == 2) {
                    telemetry.addData("Decelerating", accelState);
                    ramppf = ramppf - 0.05;
                    if (ramppf < 0) {
                        ramppf = 0;
                    }
                } else {
                    telemetry.addData("in weird accel state", accelState);
                }
            }


            an = getAngle();
            //for(DcMotor m : du.motorList ) {
            //   a = m.getCurrentPosition();

            //}

            telemetry.addData("time: ",getRuntime());

            telemetry.update();
            idle();


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
