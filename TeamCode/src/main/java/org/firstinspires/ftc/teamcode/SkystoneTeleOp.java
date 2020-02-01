package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SkystoneTeleOp (Blocks to Java)", group = "")
public class SkystoneTeleOp extends LinearOpMode {

  private Servo leftClaw;
  private Servo intake;
  private Servo rightClaw;
  private DcMotor frontLeft;
  private DcMotor backLeft;
  private DcMotor frontRight;
  private DcMotor backRight;
  private DcMotor linearSlide;
  private boolean isClawDown;
  private Servo capstone;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    leftClaw = hardwareMap.servo.get("leftClaw");
    intake = hardwareMap.servo.get("intake");
    rightClaw = hardwareMap.servo.get("rightClaw");
    frontLeft = hardwareMap.dcMotor.get("frontLeft");
    backLeft = hardwareMap.dcMotor.get("backLeft");
    frontRight = hardwareMap.dcMotor.get("frontRight");
    backRight = hardwareMap.dcMotor.get("backRight");
    linearSlide = hardwareMap.dcMotor.get("linearSlide");
    capstone = hardwareMap.servo.get("capstone");

    // Put initialization blocks here.
    leftClaw.setPosition(1);
    intake.setPosition(0);
    isClawDown = true;
    rightClaw.setPosition(0);
    capstone.setPosition(1);
    boolean isBPressed = false;
    boolean isAPressed = false;
    ElapsedTime timeSinceLastPress = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    waitForStart();
    if (opModeIsActive()) {
      double wheelsPowerFactor = 0.6;
      timeSinceLastPress.reset();
      while (opModeIsActive()) {
        double drive = gamepad1.left_stick_y * wheelsPowerFactor;
        double strafe = -(gamepad1.left_stick_x * wheelsPowerFactor);
        double rotate = gamepad1.right_stick_x * wheelsPowerFactor;
        telemetry.addData("Rotate Value", gamepad1.right_stick_x * wheelsPowerFactor);
        frontLeft.setPower(drive - (strafe - rotate));
        backLeft.setPower(drive + strafe + rotate);
        frontRight.setPower(-(drive + (strafe - rotate)));
        backRight.setPower(-(drive - (strafe + rotate)));
        //intake.setPosition(-gamepad2.right_stick_y);
        moveIntakeWithBumper(timeSinceLastPress);
        if (gamepad1.left_trigger == 1) {
          strafe_left(wheelsPowerFactor * 1.5);
        }
        if (gamepad1.right_trigger == 1) {
          strafe_right(wheelsPowerFactor * 1.5);
        }
        if (gamepad1.dpad_down && timeSinceLastPress.milliseconds() >= 1000) {
          wheelsPowerFactor = wheelsPowerFactor - 0.1;
          timeSinceLastPress.reset();
        }
        if (gamepad1.dpad_up && timeSinceLastPress.milliseconds() >= 1000) {
          wheelsPowerFactor = wheelsPowerFactor + 0.1;
          timeSinceLastPress.reset();
        }
        if (gamepad2.b && timeSinceLastPress.milliseconds() >= 750) {
          if (isBPressed == false) {
            leftClaw.setPosition(1);
            rightClaw.setPosition(0);
            isBPressed = true;
          } else {
            leftClaw.setPosition(0);
            rightClaw.setPosition(1);
            isBPressed = false;
          }
          timeSinceLastPress.reset();
        }
        if (gamepad2.a && timeSinceLastPress.milliseconds() >= 750) {
          if (isAPressed == false) {
            capstone.setPosition(0);
            isAPressed = true;
          } else {
            capstone.setPosition(1);
            isAPressed = false;
          }
          timeSinceLastPress.reset();
        }

        linearSlide.setPower(gamepad2.left_stick_y * .75);
        telemetry.addData("Linear Slide", linearSlide.getPower());
        telemetry.addData("testclaw", leftClaw.getPosition());
        telemetry.addData("Servo Position", intake.getPosition());
        telemetry.addData("change number", 6.1);
        telemetry.addData("Wheel power factor:", wheelsPowerFactor);
        telemetry.addData("capstone position", capstone.getPosition());
        telemetry.update();
      }
    }
  }

  private void moveIntakeWithBumper(ElapsedTime timeSinceLastPress) {
    if (gamepad2.right_bumper && timeSinceLastPress.milliseconds() >= 750) {
      if (isClawDown == false) {
        intake.setPosition(0);
        isClawDown = true;
      } else {
        intake.setPosition(1);
        isClawDown = false;
      }
      timeSinceLastPress.reset();
    }

  }
  /**
   * Describe this function...
   */
  private void strafe_right(double power) {
    set_power(0, -power, 0);
  }

  /**
   * Describe this function...
   */
  private void strafe_left(double power) {
    set_power(0, power, 0);
  }

  /**
   * Describe this function...
   */
  private void set_power(double drive, double strafe, double rotate) {
    frontLeft.setPower(-(drive + strafe + rotate));
    backLeft.setPower(-(drive - (strafe - rotate)));
    frontRight.setPower(drive - (strafe + rotate));
    backRight.setPower(drive + (strafe - rotate));
  }
}
