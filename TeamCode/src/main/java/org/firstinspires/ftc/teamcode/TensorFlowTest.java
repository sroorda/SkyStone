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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;


/**
 * Autonomous sample of moving with time vs moving with encoders
 *
 * @author Sylvianne J Rodgers
 */

@TeleOp(name="TeleOpTest", group="Linear Opmode")
public class TensorFlowTest extends LinearOpMode {
    public final static double SPEED = 0.75;

    public static double TICKS_PER_CM = 17.1;

    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private Servo intake = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor linearSlide = null;
    List<DcMotor> motorList = new ArrayList<DcMotor>();


    double powerFactor = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized v5");
        telemetry.update();

        DriveUtility du = new DriveUtility(hardwareMap, telemetry, this);
        //////////////////////////////////////
        waitForStart();

        if (opModeIsActive()) {

            du.tensorFlowBlue();
            sleep(500000);




            telemetry.update();

        }
    }
}



