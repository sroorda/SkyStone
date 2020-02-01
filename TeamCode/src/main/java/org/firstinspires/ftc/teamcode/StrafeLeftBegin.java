package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="StrafeLeftBegin", group="Linear Opmode")
@Disabled
public class StrafeLeftBegin extends LinearOpMode {
    public void runOpMode() {
        telemetry.addData("change number", 5);
        telemetry.addData("Status", "Initialized v7");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);


        waitForStart();

        if (opModeIsActive()) {
            sleep(5000);
            du.strafeLeftDistance(20,1.0, true);

        }
    }
}

