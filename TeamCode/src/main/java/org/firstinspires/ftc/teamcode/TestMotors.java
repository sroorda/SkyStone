package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TestMotors", group="Pushbot")
@Disabled
public class TestMotors extends LinearOpMode {

    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private double powerFactor = 1;

    @Override
    public void runOpMode() {

        backLeft  = hardwareMap.get(DcMotor.class, "backleft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        backRight  = hardwareMap.get(DcMotor.class, "backright");
        frontRight  = hardwareMap.get(DcMotor.class, "frontright");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            setPower(1, 0, 0);
        }
    }
    
    private void setPower(double drive, double strafe, double rotate) {
        // the math here is dependent on the robot and the direction of the motors
        // this is a basic sample, I would lift the proper formulas from their code
        frontLeft.setPower(powerFactor*(-drive + strafe + rotate));
        backLeft.setPower(powerFactor*(-drive - strafe - rotate));
        frontRight.setPower(powerFactor*(drive + strafe + rotate));
        backRight.setPower(powerFactor*(drive - strafe - rotate));
    }
}
