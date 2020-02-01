package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="StrafeRightBegin", group="Linear Opmode")
@Disabled
public class StrafeRightBegin extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("change number", 5);
        telemetry.addData("Status", "Initialized v7");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);


        waitForStart();

        if (opModeIsActive()) {
            sleep(5000);
            du.strafeRightDistance(20, 1.0,true);

        }
    }
}
