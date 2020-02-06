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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="CoachWTests", group="Linear Opmode")
//@Disabled
public class CoachWrightAutoTests extends LinearOpMode {

    public final static double SPEED = 0.75;


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Ac9/DqT/////AAABmWMwXGTjAUE3umMiPxGQDekSkHssqfs1px7rE0Lb51sJTncsz5jPFfDXlXFJTwO+kp17qkzL7FMszaUkQjDcril6zxU+QBCrl5oPMPOu6kHoc/5KEvqM31PG2/MIzMF4rIM9mQFw9f5Y20q13TcmDzA1RwDGdrmaGO8nv+2tid9PyfIv4s6GCdspdmSDLRq7shREVfC81Wli2bMoNI49kZNj3bydsSSIFoMBV+PX2FVvpnWkqQ2OjZiWR4h08/c1VdaiBqrN5AH1Lz5LYlM0Hr0M/ZCgQdC2rQVxNfBV1pY6XfzuTeYoksCFbUxrSBs9nvSwz4/0LPVIMmmvYS57kjgpDv0CvbQ5p/svOvuMNuvG";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized v6");
        telemetry.update();
        DriveUtility du = new DriveUtility(hardwareMap,telemetry,this);
        du.moveIntake(DriveUtility.CLAW_OPEN);
        du.moveLeftClawAndRightClaw(DriveUtility.FOUNDATION_CLAW_OPEN);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        if (opModeIsActive()) {
            du.log("Move to get good image", "");
            du.moveWithEncoder(24, .3, true);
            sleep(100);

            du.log("Start skystone recognition", "");
            if (tfod != null) {
                du.log("Tensorflow is initialized", "");
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                double startTime = getRuntime();
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                du.log("First # Object Detected", "" + updatedRecognitions.size());
                if( updatedRecognitions.size() < 2 ) {
                    sleep(300);
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                }
                if (updatedRecognitions != null) {
                    du.log("# Object Detected", "" + updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    Recognition skystone = null;
                    boolean foundSkystone = false;
                    double leftCoord = 1000; // be more intelligent with this value
                    //Recognition leftMostStone = null;
                    for (Recognition recognition : updatedRecognitions) {
                        du.log(String.format("label (%d)", i), recognition.getLabel() + ", " + recognition.getConfidence());
                        du.log(String.format("  left,top (%d)", i), recognition.getLeft() + "," + recognition.getTop());
                        du.log(String.format("  right,bottom (%d)", i), recognition.getRight() + ", " + recognition.getBottom());

                        if( recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            foundSkystone = true;
                            skystone = recognition;
                        }

                        if( recognition.getLeft() < leftCoord ) {
                            leftCoord = recognition.getLeft();
                        }
                    }

                    if( foundSkystone ) {
                        // first or second position
                        du.log("Skystone", "1st or 2nd Position");
                        if( skystone.getLeft() == leftCoord ) {
                            du.log("Skystone Position", "2");
                        }
                        else {
                            du.log("Skystone Position", "1");
                        }
                    }
                    else {
                        // third position
                        du.log("Skystone Position", "3 (maybe)");
                    }
                    du.log("Time", "" + (getRuntime() - startTime));
                    telemetry.update();
                }
                else {
                    du.log("Recognitions is NULL", "");
                }
            }
            else {
                du.log("Tensorflow not initialized", "");
            }
        }

        sleep(5000);
        du.log("Shutting down Tensorflow", "");
        if (tfod != null) {
            tfod.shutdown();
            du.log("Shutdown complete", "");
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


}
