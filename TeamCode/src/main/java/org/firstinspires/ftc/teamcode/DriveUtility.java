package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

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

    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontRight = null;
    private Servo intake = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DcMotor linearSlide = null;
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
    
    // MOVE TO POSITION FUNCTIONS



    public void moveWithEncoder(double distance, double speed){
        moveWithEncoder(distance, speed, false);

    }



    public void moveWithEncoder(double distance, double speed, boolean fullSpeed) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        int roundedDistance = (int)Math.round(distance * TICKS_PER_CM);
        backLeft.setTargetPosition(-roundedDistance);
        frontLeft.setTargetPosition(-roundedDistance);
        backRight.setTargetPosition(roundedDistance);
        frontRight.setTargetPosition(roundedDistance);
        
        
        runMotorsUntilPositionRampSpeed(speed, 0, 0, fullSpeed, roundedDistance, STATE_MOVE);
    }

    public void strafeRightDistance(double distance, double speed) {
        strafeRightDistance(distance, speed, false);
    }

    public void strafeRightDistance(double distance, double speed, boolean fullSpeed) {
        for(DcMotor m : motorList ) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            //m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        setMotorSpeeds(1, flPower, blPower, frPower, brPower);
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setMotorSpeeds(1, 0, 0, 0, 0);

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
        double flPower = -drive - strafe + rotate;
        double blPower = -drive + strafe - rotate;
        double frPower = drive - strafe + rotate;
        double brPower = drive + strafe - rotate;
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
        while( opMode.opModeIsActive() && !reachedDistance(targetDistance, backLeft, frontLeft, backRight, frontRight)) {
            double angle = getAngle();

            if( moveState != STATE_MOVE ) {
                if (angle > centerAngle) {
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
                }
            }
            log("Angle", "" + angle);
            log("Correction value", backCorrection + ", " + frontCorrection);
            flPower = flPower * frontCorrection;
            frPower = frPower * frontCorrection;
            brPower = brPower * backCorrection * backPowerCorrect;
            blPower = blPower * backCorrection * backPowerCorrect;
            log("power", blPower + ", " + flPower + ", " + brPower + ", " + frPower);

            if (speedFactor < 1){
                speedFactor += AMOUNT_INCREASED;
                setMotorSpeeds(speedFactor, flPower, blPower, frPower, brPower);
            }
            else {
                //if( strafe != 0 ) {
                    //setMotorSpeeds(speedFactor, flPower, blPower * 1.08, frPower, brPower * 1.08);
                //}
            }
            opMode.sleep(50);
            opMode.idle();
            //log("busy:", backLeft.isBusy() + "," + frontLeft.isBusy()+ "," + backRight.isBusy()+ "," + frontRight.isBusy());
            //log("position:", backLeft.getCurrentPosition() + "," + frontLeft.getCurrentPosition()+ "," + backRight.getCurrentPosition()+ "," + frontRight.getCurrentPosition() );
            //log("power:", backLeft.getPower() + "," + frontLeft.getPower()+ "," + backRight.getPower()+ "," + frontRight.getPower() );
            telemetry.update();

        }
        for(DcMotor m : motorList ) {
            //if( !m.isBusy() ) {

                m.setPower(0);
            //}
        }
    }


    private void setMotorSpeeds(double speedFactor, double flPower, double blPower, double frPower, double brPower) {
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
    public void setPower(double drive, double strafe, double rotate) {
        frontLeft.setPower(powerFactor*(-drive + strafe + rotate));
        backLeft.setPower(powerFactor*(-drive - strafe - rotate));
        frontRight.setPower(powerFactor*(drive - strafe + rotate));
        backRight.setPower(powerFactor*(drive + strafe - rotate));
        log("motor powers", frontLeft.getPower() + "," + backLeft.getPower() + "," + frontRight.getPower() + "," + backRight.getPower() );
    }


    protected void initHardware() {
        // read all the motors from the map
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        motorList.add(backLeft);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        motorList.add(frontLeft);
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        motorList.add(backRight);
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        motorList.add(frontRight);

        linearSlide = hardwareMap.dcMotor.get("linearSlide");

        intake = hardwareMap.servo.get("intake");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

}
    
    
    
    
    
    

