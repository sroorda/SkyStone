package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class DriveUtility {
    public final static int CLAW_OPEN = 1;
    public final static int CLAW_CLOSE = 0;

    public final static int FOUNDATION_CLAW_OPEN = 0;
    public final static int FOUNDATION_CLAW_CLOSE = 1;

    public final static double STRAFE_LEFT_BACK_POWER = 1.0085;
    public final static double STRAFE_RIGHT_BACK_POWER = 1.025;

    public final static double STRAFE_LEFT_ANGLE_CORRECT = 0.001;
    public final static double STRAFE_RIGHT_ANGLE_CORRECT = 0.003;

    public final static int STATE_MOVE = 0;
    public final static int STATE_STRAFE_LEFT = 1;
    public final static int STATE_STRAFE_RIGHT = 2;

    public static double TICKS_PER_CM = 17.1;
    public static double LINEAR_TICKS_PER_CM = 45;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "Ac9/DqT/////AAABmWMwXGTjAUE3umMiPxGQDekSkHssqfs1px7rE0Lb51sJTncsz5jPFfDXlXFJTwO+kp17qkzL7FMszaUkQjDcril6zxU+QBCrl5oPMPOu6kHoc/5KEvqM31PG2/MIzMF4rIM9mQFw9f5Y20q13TcmDzA1RwDGdrmaGO8nv+2tid9PyfIv4s6GCdspdmSDLRq7shREVfC81Wli2bMoNI49kZNj3bydsSSIFoMBV+PX2FVvpnWkqQ2OjZiWR4h08/c1VdaiBqrN5AH1Lz5LYlM0Hr0M/ZCgQdC2rQVxNfBV1pY6XfzuTeYoksCFbUxrSBs9nvSwz4/0LPVIMmmvYS57kjgpDv0CvbQ5p/svOvuMNuvG";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private Servo intake = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor linearSlide = null;
    private DistanceSensor leftSensor = null;
    private DistanceSensor rightSensor = null;
    List<DcMotor> motorList = new ArrayList<DcMotor>();
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;

    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;



    double powerFactor = 1;

    public DriveUtility(HardwareMap map, Telemetry telem, LinearOpMode linearOpMode) {
        hardwareMap = map;
        telemetry = telem;
        opMode = linearOpMode;
        initHardware();
        initIMU();
        initEncoders();
        initTensorFlow();
    }

    // INITIATING ENCODERS
    protected void initEncoders() {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(0);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())
        {
            opMode.sleep(50);
            opMode.idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void initTensorFlow() {


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

    }

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

    //Tensor Flow
    public int tensorFlow() {
        int position = 1;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size() < 2){
                opMode.sleep(300);
                updatedRecognitions = tfod.getUpdatedRecognitions();
                log("recognition", "2nd attempt");
            }
            if (updatedRecognitions == null || updatedRecognitions.size() < 2){
                opMode.sleep(300);
                updatedRecognitions = tfod.getUpdatedRecognitions();
                log("recognition", "3rd attempt");
            }

            if (updatedRecognitions != null) {
                //captureFrameToFile(updatedRecognitions);
                log("# Object Detected", "" + updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                Recognition skystone = null;
                Recognition leftmost = null;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                        skystone = recognition;
                        telemetry.addData("Found Skystone", "");
                    }

                    if (leftmost == null){
                        leftmost = recognition;
                    }
                    else{
                        if (recognition.getLeft() < leftmost.getLeft()){
                            leftmost = recognition;
                        }

                    }
                }
                if (skystone != null ){
                    log("Skystone", "" + skystone.getLeft());
                    log("Leftmost", "" + leftmost.getLeft());
                }
                else {
                    log("Skystone", "NULL");
                    log("Leftmost", "NULL");
                }

                if (updatedRecognitions.size() <= 1){
                    log("Skystone Position", "1 (NO OBJECTS FOUND)");
                    position = 1;
                }
                else if (skystone == null){
                    log("Skystone Position", "3");
                    telemetry.addData("Skystone is in position 3", "");
                    position = 3;
                }
                else if (skystone.getLeft() == leftmost.getLeft()){
                    log("Skystone Position", "2");
                    telemetry.addData("Skystone is in position 2", "");
                    position = 2;
                }
                else {
                    log("Skystone Position", "1");
                    telemetry.addData("Skystone is in position 1", "");
                    position = 1;
                }
                telemetry.update();
            }
            else{
                log("Skystone = Null", "");
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }

        return position;
    }

    //first stone
    public void firstStone(boolean isRed){
        if(isRed){

        }
    }

    // MOVE TO POSITION FUNCTIONS

    public void moveWithEncoder(double distance, double speed){
        moveWithEncoder(distance, speed, false);

    }



    public void moveWithEncoder(double distance, double speed, boolean fullSpeed) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        int roundedDistance = (int)Math.round(distance * TICKS_PER_CM);
/*
        backLeft.setTargetPosition(-roundedDistance);
        frontLeft.setTargetPosition(-roundedDistance);
        backRight.setTargetPosition(roundedDistance);
        frontRight.setTargetPosition(roundedDistance);
*/

        runMotorsUntilPositionRampSpeed(speed, 0, 0, fullSpeed, roundedDistance, STATE_MOVE);
    }

    public void strafeRightDistance(double distance, double speed) {
        strafeRightDistance(distance, speed, false);
    }

    public void strafeRightDistance(double distance, double speed, boolean fullSpeed) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        int roundedDistance = (int)Math.round(distance * TICKS_PER_CM);
        int roundedFrontDistance = (int)Math.round(distance * TICKS_PER_CM);// * 0.975);
        //backLeft.setTargetPosition(-roundedDistance);
        //frontLeft.setTargetPosition(roundedFrontDistance);
        //backRight.setTargetPosition(-roundedDistance);
        //frontRight.setTargetPosition(roundedFrontDistance);


        runMotorsUntilPositionRampSpeed(0, -speed, 0, fullSpeed, roundedDistance, STATE_STRAFE_RIGHT);
    }

    public void strafeLeftDistance(double distance, double speed) {
        strafeLeftDistance(distance, speed, false);
    }

    public void strafeLeftDistance(double distance, double speed, boolean fullSpeed) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        int roundedDistance = (int)Math.round(distance * TICKS_PER_CM);
        int roundedFrontDistance = (int)Math.round(distance * TICKS_PER_CM);//* 0.975);
        //backLeft.setTargetPosition(roundedDistance);
        //frontLeft.setTargetPosition(-roundedFrontDistance);
        //backRight.setTargetPosition(roundedDistance);
        //frontRight.setTargetPosition(-roundedFrontDistance);

        runMotorsUntilPositionRampSpeed(0, speed, 0, fullSpeed, roundedDistance, STATE_STRAFE_LEFT);
    }

    public void rotateRight (long milliseconds, double rotate) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double flPower = rotate;
        double blPower = rotate;
        double frPower = rotate;
        double brPower = rotate;
        log("ROTATION ANGLE BEFORE", "" + getAngle());
        setMotorSpeeds(1, flPower, blPower, frPower, brPower);
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setMotorSpeeds(1, 0, 0, 0, 0);
        log("ROTATION ANGLE AFTER", "" + getAngle());

    }

    public void rotateLeft (long milliseconds, double rotate) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double flPower = -rotate;
        double blPower = -rotate;
        double frPower = -rotate;
        double brPower = -rotate;
        setMotorSpeeds(1, flPower, blPower, frPower, brPower);
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setMotorSpeeds(1, 0, 0, 0, 0);

    }

    // RESET ANGLE FUNCTION
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    //ROTATE FUNCTION
    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double rotate)
    {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        double  flPower, blPower, frPower, brPower;

        // restart imu movement tracking.
        //resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            flPower = rotate;
            blPower = rotate;
            frPower = rotate;
            brPower = rotate;
        }
        else if (degrees > 0)
        {   // turn left.
            flPower = -rotate;
            blPower = -rotate;
            frPower = -rotate;
            brPower = -rotate;
        }
        else return;

        // set power to rotate.
        setMotorSpeeds(1, flPower, blPower, frPower, brPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            log("rotate", "turning clockwise (right, -)");
            // On right turn we have to get off zero first.
            double angle = getAngle();
            while (opMode.opModeIsActive() && angle == 0) {
                log("rotate clockwise,0", "getAngle=" +getAngle());
                angle = getAngle();
            }

            while (opMode.opModeIsActive() && angle > degrees) {
                log("rotate clockwise, not 0", "getAngle=" +getAngle());
                angle = getAngle();
            }
        }
        else {   // left turn.
            log("rotate", "turning counter-clockwise (left, +)");
            while (opMode.opModeIsActive() && getAngle() < degrees) {}
        }

        // turn the motors off.
        setMotorSpeeds(1,0,0,0,0);

        log("rotate", "end of method angle: " + getAngle());

        // reset angle tracking on new heading.
        resetAngle();
    }

    // SERVO FUNCTIONS
    public void moveIntake(double position) {
        intake.setPosition(position);
    }

    public void moveLeftClawAndRightClaw (double position) {
        leftClaw.setPosition(position);
        rightClaw.setPosition(1-position);

    }

    // LINEAR SLIDE FUNCTION

    public void moveLinearSlideWithTime(long milliseconds) {

        linearSlide.setPower(0.75);
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        linearSlide.setPower(0);


    }

    public void moveLinearSlideWithTimeNegative(long milliseconds) {

        linearSlide.setPower(-0.75);
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        linearSlide.setPower(0);

    }

    public void moveLinearSlideWithEncoders(double distance) {

        log("function has been called","");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int roundedDistance = (int)Math.round(distance * LINEAR_TICKS_PER_CM);
        linearSlide.setTargetPosition(roundedDistance*-1);

        linearSlide.setPower(0.5);
        log( "before linear slide position", "" + linearSlide.getCurrentPosition());
        log( "before linear slide target", "" + linearSlide.getTargetPosition());
        while (opMode.opModeIsActive() && linearSlide.getCurrentPosition() >= linearSlide.getTargetPosition()+20 ) {
            log( "linear slide position", "" + linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0);

    }

    public void moveLinearSlideWithRunUsingEncoders(double distance) {

        log("function has been called","");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double targetDistance = -distance * LINEAR_TICKS_PER_CM;

        linearSlide.setPower(-0.5);
        log( "before linear slide position", "" + linearSlide.getCurrentPosition());
        log( "before linear slide target", "" + targetDistance);
        while (opMode.opModeIsActive() && linearSlide.getCurrentPosition() >= targetDistance ) {
            log( "linear slide position", "" + linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0);

    }

    public void moveLinearSlideWithRunUsingEncodersDown(double distance) {

        log("function has been called","");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double targetDistance = distance * LINEAR_TICKS_PER_CM;

        linearSlide.setPower(0.5);
        log( "before linear slide position", "" + linearSlide.getCurrentPosition());
        log( "before linear slide target", "" + targetDistance);
        while (opMode.opModeIsActive() && linearSlide.getCurrentPosition() <= targetDistance ) {
            log( "linear slide position", "" + linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0);

    }



    // RUN TO POSITION MAIN FUNCTION
    private static final double START_SPEED = 0.1; // Ramping value
    private static final double AMOUNT_INCREASED = 0.1; // Ramping value
    //private static final double STRAFE_FACTOR = 0.001; //0.003; // How much correct for angle per check
    //private static final double BACK_POWER_CORRECT = 1.0085;//1.025; // Overall correction

    protected void runMotorsUntilPositionRampSpeed(double drive, double strafe, double rotate, boolean fullSpeed, double targetDistance, int moveState) {
        double angleCorrect = 1;
        double backPowerCorrect = 1;

        if (moveState == STATE_STRAFE_RIGHT) {
            angleCorrect = STRAFE_RIGHT_ANGLE_CORRECT;
            backPowerCorrect = STRAFE_RIGHT_BACK_POWER;
        }
        else if (moveState == STATE_STRAFE_LEFT ){
            angleCorrect = STRAFE_LEFT_ANGLE_CORRECT;
            backPowerCorrect = STRAFE_LEFT_BACK_POWER;
        }

        //double flPower = -drive + strafe + rotate;
        //double blPower = -drive - strafe - rotate;
        double flPower = drive - strafe + rotate;
        double blPower = drive + strafe - rotate;
        double frPower = -drive - strafe + rotate;
        double brPower = -drive + strafe - rotate;
        double backCorrection = 1;
        double frontCorrection = 1;

        double speedFactor = START_SPEED;
        if( fullSpeed ) {
            speedFactor = 1;
        }
        setMotorSpeeds(speedFactor, flPower, blPower, frPower, brPower);

        // this should turn off each motor as it reaches its position, which means if one is off
        // it will run slightly longer or faster than others
        //while( opMode.opModeIsActive() && (backLeft.isBusy() || frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy())) {
        double centerAngle = getAngle();
        double angleError = 0;
        double pTerm = 0;
        double cumulativeAngleError = 0;

        while( opMode.opModeIsActive() && !reachedDistance(targetDistance, backLeft, frontLeft, backRight, frontRight)) {
            double angle = getAngle();

            if( moveState != STATE_MOVE ) {
                angleError = angle - centerAngle;
                cumulativeAngleError += angleError;
                if (moveState == STATE_STRAFE_RIGHT) {
                    pTerm = 0.001;
                    frontCorrection = 1 +(pTerm * angleError);
                    backCorrection = 1 -(pTerm * angleError);

                } else if (moveState == STATE_STRAFE_LEFT ){
                    pTerm = 0.0001;
                    frontCorrection = 1 -(pTerm * angleError);
                    backCorrection = 1 +(pTerm * angleError);
                }




                /* if (angle > centerAngle) {
                    // increase the back wheel power and decrease the front wheel power
                    if (moveState == STATE_STRAFE_RIGHT) {
                        backCorrection = backCorrection* (1 - angleCorrect);
                        frontCorrection = frontCorrection * (1 + angleCorrect);
                    } else if (moveState == STATE_STRAFE_LEFT ){
                        backCorrection = backCorrection * (1 + angleCorrect);
                        frontCorrection = frontCorrection * (1 - angleCorrect);
                    }
                } else if (angle < centerAngle) {
                    // decrease the back wheel power and increase the front wheel power
                    if (moveState == STATE_STRAFE_RIGHT) {
                        backCorrection = backCorrection * (1 + angleCorrect);
                        frontCorrection = frontCorrection * (1 - angleCorrect);
                    } else if (moveState == STATE_STRAFE_LEFT){
                        //
                        backCorrection = backCorrection * (1 - angleCorrect);
                        frontCorrection = frontCorrection * (1 + angleCorrect);
                    }
                } */
            }
            //log("Angle", "" + angle);
            //log("Correction value", backCorrection + ", " + frontCorrection);
            flPower = flPower * frontCorrection;//.998;
            frPower = frPower * frontCorrection;//.998;
            brPower = brPower * backCorrection; // * backPowerCorrect;
            blPower = blPower * backCorrection; // * backPowerCorrect;


            if (speedFactor < 1){
                speedFactor += AMOUNT_INCREASED;
            }

            setMotorSpeeds(speedFactor, flPower, blPower, frPower, brPower);
            //opMode.sleep(50);
            opMode.idle();
            //log("busy:", backLeft.isBusy() + "," + frontLeft.isBusy()+ "," + backRight.isBusy()+ "," + frontRight.isBusy());
            //log("position:", backLeft.getCurrentPosition() + "," + frontLeft.getCurrentPosition()+ "," + backRight.getCurrentPosition()+ "," + frontRight.getCurrentPosition() );
            //log("power:", backLeft.getPower() + "," + frontLeft.getPower()+ "," + backRight.getPower()+ "," + frontRight.getPower() );
            telemetry.update();

        }

        setMotorSpeeds(0, 0, 0, 0, 0);
    }


    public void setMotorSpeeds(double speedFactor, double flPower, double blPower, double frPower, double brPower) {
        if (speedFactor > 1) {
            speedFactor = 1;
        } else if (speedFactor < 0) {
            speedFactor = 0;
        }
        //log("power bl fl br fr: ", blPower + ", " + flPower + ", " + brPower + ", " + frPower);
        log("power: ", String.format("bl %.5f fl %.5f br %.5f fr %.5f",blPower,flPower,brPower,frPower));
        frontLeft.setPower(speedFactor * flPower);
        backRight.setPower(speedFactor * blPower);
        frontRight.setPower(speedFactor * frPower);
        backLeft.setPower(speedFactor * brPower);
        //log("motor powers", frontLeft.getPower() + "," + backLeft.getPower() + "," + frontRight.getPower() + "," + backRight.getPower() );
    }

    public static final int ENCODERS_CLOSE_ENOUGH = 20;
    private boolean busy(DcMotor...ms) {
        int total = 0;
        for(DcMotor m: ms) {
            if(m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                final int t = Math.abs(m.getTargetPosition());
                total += Math.max(0, t - c);
            }
        }

        if( total <= ENCODERS_CLOSE_ENOUGH ) {
            log("busy()", "REACHED TARGET DISTANCE!");
            log("position:", backLeft.getCurrentPosition() + "," + frontLeft.getCurrentPosition()+ "," + backRight.getCurrentPosition()+ "," + frontRight.getCurrentPosition() );
        }
        return total > ENCODERS_CLOSE_ENOUGH;
    }

    private boolean reachedDistance(double targetDistance, DcMotor...ms) {
        double total = 0;
        for(DcMotor m: ms) {
            total += Math.abs(m.getCurrentPosition());
        }
        double average = total / 4;
        return (average + ENCODERS_CLOSE_ENOUGH >= Math.abs(targetDistance));
    }

    private boolean averageBusy(DcMotor...ms) {
        int total = 0;
        for(DcMotor m: ms) {
            if(m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                final int t = Math.abs(m.getTargetPosition());
                total = total + Math.max(0, t - c);
            }
        }
        total = total / ms.length;

        if( total <= ENCODERS_CLOSE_ENOUGH ) {
            log("busy()", "REACHED TARGET DISTANCE!");
            log("position:", backLeft.getCurrentPosition() + "," + frontLeft.getCurrentPosition()+ "," + backRight.getCurrentPosition()+ "," + frontRight.getCurrentPosition() );
        }
        return total > ENCODERS_CLOSE_ENOUGH;
    }

    private boolean averageMoved(DcMotor...ms) {
        int total = 0;
        double target = 100000;
        for(DcMotor m: ms) {
            target = Math.abs(m.getTargetPosition());
            if(m.isBusy()) {
                final int c = Math.abs(m.getCurrentPosition());
                total = total + c;
            }
        }
        total = total / ms.length;
        //log("total, target", total + ", " + target);

        if( total >= target ) {
            log("busy()", "REACHED TARGET DISTANCE!");
            log("position:", backLeft.getCurrentPosition() + "," + frontLeft.getCurrentPosition()+ "," + backRight.getCurrentPosition()+ "," + frontRight.getCurrentPosition() );
        }
        return total >= target;
    }




    // SET POWER FUNCTION
    public void setPower() {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        frontLeft.setPower(1);
        opMode.sleep(1000);
        backLeft.setPower(1);
        opMode.sleep(1000);
        frontRight.setPower(1);
        opMode.sleep(1000);
        backRight.setPower(1);
        opMode.sleep(1000);
    }


    protected void initHardware() {
        // read all the motors from the map
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        motorList.add(backLeft);
        frontRight = hardwareMap.get(DcMotor.class, "frontLeft");
        motorList.add(frontRight);
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        motorList.add(backRight);
        frontLeft  = hardwareMap.get(DcMotor.class, "frontRight");
        motorList.add(frontLeft);

        linearSlide = hardwareMap.dcMotor.get("linearSlide");

        intake = hardwareMap.servo.get("intake");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftSensor = hardwareMap.get(DistanceSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightSensor");
    }

    protected void log(String caption, String message) {
        telemetry.addData(caption, message);
        RobotLog.a(opMode.getRuntime() + " [BBTC]" + caption + ": " + message);
        //telemetry.update();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
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

    /**
     * Method retrieved from
     * https://github.com/FIRST-Tech-Challenge/SkyStone/blob/14ac54abab5dbf05fe2f2ac7ad7e601e7dd710bd/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVuforiaNavigationWebcam.java#L438
     *
     */

    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the camera is actually seeing and so assist in camera
     * aiming and alignment.
     */
    void captureFrameToFile(final List<Recognition> recognitions) {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            RobotLog.a("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee("CaptureFrameToFile", e, "exception in captureFrameToFile()");
                    }

                    // write out the recognitions and their details in a file so we can draw the bounding boxes later
                    File recogFile = new File(captureDirectory, String.format(Locale.getDefault(), "Recognitions-%d.png", captureCounter++));
                    StringBuilder sb = new StringBuilder();
                    for( Recognition r : recognitions ) {
                        sb.append(String.format("%d,", r.getLabel()));
                        sb.append(String.format("(%d,), ", "%.03f , %.03f", r.getLeft(), r.getTop()));
                        sb.append(String.format("(%d,), ", "%.03f , %.03f", r.getRight(), r.getBottom()));
                        sb.append("\n");
                    }
                    try {
                        FileOutputStream outputStream = new FileOutputStream(recogFile);
                        try {
                            outputStream.write(sb.toString().getBytes());
                            outputStream.flush();
                        } finally {
                            outputStream.close();
                            RobotLog.a("captured %s", recogFile.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee("CaptureFrameToFile", e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }

    public double distSensorsAngle(){

        double leftSensorDist = -1;
        double rightSensorDist = -1;
        double surfaceAngle = 0;
        double adjacentDist = 36; // in cm
        double oppositeDist = 0;

        leftSensorDist = leftSensor.getDistance(DistanceUnit.CM);
        rightSensorDist = rightSensor.getDistance(DistanceUnit.CM);

        if((leftSensorDist > 50) || (rightSensorDist > 50)) {
            surfaceAngle = Double.NaN;
        }
        else {
            oppositeDist = rightSensorDist - leftSensorDist;
            surfaceAngle = Math.toDegrees(Math.atan(oppositeDist/adjacentDist));

        }

        return surfaceAngle;

        //telemetry.addData("opp", String.format("%.01f cm", oppositeDist));
        //telemetry.addData("adj", String.format("%.01f cm", adjacentDist));
        //telemetry.addData("L range", String.format("%.01f cm", leftSensorDist));
        //telemetry.addData("R range", String.format("%.01f cm", rightSensorDist));
        //telemetry.addData("  angle", String.format("%.01f deg", surfaceAngle));

    }

    public void sensorCorrection( double speed, double fudgeAngle, long timeToRotate) {
        double angleWeAreOffBy = distSensorsAngle();

        if (angleWeAreOffBy < fudgeAngle) {
            rotateRight(timeToRotate, speed);
            log("ANGLE", "" + angleWeAreOffBy);
            //opMode.sleep(1000);

        } else if (angleWeAreOffBy > fudgeAngle) {
            rotateLeft(timeToRotate, speed);
            log("ANGLE", "" + angleWeAreOffBy);
            //opMode.sleep(1000);

        }else{
            rotateLeft(1, 0);
            log("ANGLE AFTER WE STOP", "" + angleWeAreOffBy);
            //opMode.sleep(1000);

        }
    }

    static double ANGLE_FUDGE = 1; //coach

    public void angleCorrect () {
        double ourAngle = distSensorsAngle();


        log("[angleCorrectDistSensor]", "" + ourAngle);
        if (ourAngle < ANGLE_FUDGE) {
            while (ourAngle < -ANGLE_FUDGE) {

                rotateRight(10, 0.3);
                ourAngle = distSensorsAngle();
                log("[angleCorrectDistSensor]", "" + ourAngle);
                log("[angleCorrectDistSensor],rotate", "right");
            }
        } else if (ourAngle > ANGLE_FUDGE) {

            while (ourAngle > ANGLE_FUDGE) {
                rotateLeft(10, 0.3);
                ourAngle = distSensorsAngle();
                log("[angleCorrectDistSensor]", "" + ourAngle);
                log("[angleCorrectDistSensor],rotate", "left");
            }
        } else {

        }
    }

    static double IMU_ANGLE_FUDGE = 0.2; //coach

    public void angleCorrectIMU (double targetAngle) {
        double ourAngle = -getAngle();


        log("[angleCorrectIMU]", "" + ourAngle);
        if (ourAngle < targetAngle-IMU_ANGLE_FUDGE) {
            log("[angleCorrectIMU] need to rotate", "right");
            while (ourAngle < targetAngle-IMU_ANGLE_FUDGE ) {

                rotateRight(10, 0.3);
                ourAngle = -getAngle();
                log("[angleCorrectIMU]", "" + ourAngle);
                log("[angleCorrectIMU],rotate", "right");
            }
        } else if (ourAngle > targetAngle + IMU_ANGLE_FUDGE) {
            log("[angleCorrectIMU] need to rotate", "left");
            while (ourAngle > targetAngle + IMU_ANGLE_FUDGE) {
                rotateLeft(10, 0.3);
                ourAngle = -getAngle();
                log("[angleCorrectIMU]", "" + ourAngle);
                log("[angleCorrectIMU],rotate", "left");
            }
        } else {
            log("[angleCorrectIMU] need to rotate", "none");
        }
    }

    public void rotateTest(double targetAngle) {
        double rotationDirection = 0;
        if(targetAngle < 0) {rotationDirection = 1;}
        else if(targetAngle > 0) {rotationDirection = -1;}

        double flPower = rotationDirection;
        double blPower = rotationDirection;
        double frPower = rotationDirection;
        double brPower = rotationDirection;

        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double speedFactor = 1;
        setMotorSpeeds(speedFactor, flPower, blPower, frPower, brPower);
        double avgEncoderVals = 0;

        int c = 20;
        while(c > 0) {
            avgEncoderVals = (Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()))/4;
            log("[fix move], encoder", "" + avgEncoderVals);
            log("[fix move], angle", "" + (-getAngle()));
            try {
                Thread.sleep(10);
                } catch (InterruptedException e) {
                e.printStackTrace();
                }
            c--;
        }
        setMotorSpeeds(speedFactor, 0, 0, 0, 0);
    }

    double returnAngleFromEncoderAtFullSpeedRotate() {
        double avgEncoderVals = (Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()))/4;
        return 0.1466*avgEncoderVals + 27.73;
    }

    public void rotateViaIMUToAngleTest(double targetAngle) {
        /*for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        setMotorSpeeds(1, 1, 1, 1, 1);
        while(returnAngleFromEncoderAtFullSpeedRotate() > 40) {
            try {Thread.sleep(1);} catch (InterruptedException e) {e.printStackTrace();}
        }*/
        boolean seek = false;
        double currentAngle = -getAngle();
        double proportionalAngleDelta = Math.abs(Math.abs(currentAngle) - Math.abs(targetAngle));
        double startingAngleDelta = proportionalAngleDelta;
        double speedFactor = .3;
        int rotationDirection = 0;
        double speedFactorProportion = 0.01;
        log("[Set angle via IMU]","start");
        while(currentAngle > targetAngle + IMU_ANGLE_FUDGE || currentAngle < targetAngle - IMU_ANGLE_FUDGE) {


            if(currentAngle > targetAngle + IMU_ANGLE_FUDGE) {
                rotationDirection = -1;
                log("[angleCorrectIMU]", "" + currentAngle);
                log("[angleCorrectIMU],rotate", "left");

            }
            else if (currentAngle < targetAngle-IMU_ANGLE_FUDGE) {
                rotationDirection = 1;
                log("[angleCorrectIMU]", "" + currentAngle);
                log("[angleCorrectIMU],rotate", "right");
            }
            else
            {
                rotationDirection = 0;
                log("[angleCorrectIMU]", "" + currentAngle);
                log("[angleCorrectIMU],rotate", "shouldn't be here");
            }

            for(DcMotor m : motorList ) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            double flPower = rotationDirection;
            double blPower = rotationDirection;
            double frPower = rotationDirection;
            double brPower = rotationDirection;

            proportionalAngleDelta = Math.abs(Math.abs(currentAngle) - Math.abs(targetAngle));

            speedFactor = speedFactorProportion*proportionalAngleDelta;
            if(proportionalAngleDelta > startingAngleDelta*0.75 && !seek) {
                speedFactor = .6;
            }
            else {
                seek = true;
                if(proportionalAngleDelta > startingAngleDelta*.6) {speedFactor = 0.5;}
                else if(proportionalAngleDelta > startingAngleDelta*0.5) {speedFactor = 0.4;}
                else if(proportionalAngleDelta > startingAngleDelta*0.4) {speedFactor = 0.3;}
                else if(proportionalAngleDelta > startingAngleDelta*0.1) {speedFactor = 0.2;}
                else if(proportionalAngleDelta > startingAngleDelta*0.05) {speedFactor = 0.1;}
                else {speedFactor = 0.1;}


                /*if (speedFactor > 0.50) {
                    speedFactor = 0.5;
                }
                if (speedFactor < .1) {
                    speedFactor = 0.1;
                }*/
            }
            speedFactor = 0.4;
            setMotorSpeeds(speedFactor, flPower, blPower, frPower, brPower);
            /*try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }*/


            currentAngle = -getAngle();

        }
        setMotorSpeeds(1, 0, 0, 0, 0);
    }

    public void rotateViaIMUToAngle(double targetAngle) {
        double currentAngle = -getAngle();
        double proportionalAngleDelta = Math.abs(Math.abs(currentAngle) - Math.abs(targetAngle));
        double speedFactor = .3;
        int rotationDirection = 0;
        double speedFactorProportion = 0.01;
        log("[Set angle via IMU]","start");
        while(currentAngle > targetAngle + IMU_ANGLE_FUDGE || currentAngle < targetAngle - IMU_ANGLE_FUDGE) {


            if(currentAngle > targetAngle + IMU_ANGLE_FUDGE) {
                rotationDirection = -1;
                log("[angleCorrectIMU]", "" + currentAngle);
                log("[angleCorrectIMU],rotate", "left");

            }
            else if (currentAngle < targetAngle-IMU_ANGLE_FUDGE) {
                rotationDirection = 1;
                log("[angleCorrectIMU]", "" + currentAngle);
                log("[angleCorrectIMU],rotate", "right");
            }
            else
            {
                rotationDirection = 0;
                log("[angleCorrectIMU]", "" + currentAngle);
                log("[angleCorrectIMU],rotate", "shouldn't be here");
            }

            for(DcMotor m : motorList ) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            double flPower = rotationDirection;
            double blPower = rotationDirection;
            double frPower = rotationDirection;
            double brPower = rotationDirection;

            proportionalAngleDelta = Math.abs(Math.abs(currentAngle) - Math.abs(targetAngle));

            speedFactor = speedFactorProportion*proportionalAngleDelta;
            if(speedFactor > 0.40) {speedFactor = 0.4;}
            if(speedFactor < .1) {speedFactor = 0.1;}

            setMotorSpeeds(speedFactor, flPower, blPower, frPower, brPower);
            /*try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }*/


            currentAngle = -getAngle();

        }
        setMotorSpeeds(1, 0, 0, 0, 0);
    }

    public static double calculateSD(double numArray[])
    {
        double sum = 0.0, standardDeviation = 0.0;
        int length = numArray.length;

        for(double num : numArray) {
            sum += num;
        }

        double mean = sum/length;

        for(double num: numArray) {
            standardDeviation += Math.pow(num - mean, 2);
        }


        return Math.sqrt(standardDeviation/length);
    }
}

