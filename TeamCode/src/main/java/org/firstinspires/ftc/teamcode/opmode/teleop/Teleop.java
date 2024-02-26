package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Teleop extends LinearOpMode {
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor rotator = null;
    DcMotor lifter = null;
    CRServo launchServo = null;
    Servo grabberTilt = null;
    Servo grabberR = null;
    Servo grabberL = null;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        rotator = hardwareMap.dcMotor.get("rotator");
        lifter = hardwareMap.dcMotor.get("lifter");

        launchServo = hardwareMap.crservo.get("launcher");
        grabberTilt = hardwareMap.servo.get("grabberTilt");
        grabberR = hardwareMap.servo.get("grabberR");
        grabberL = hardwareMap.servo.get("grabberL");


        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean leftGrabberOpen = false;
        boolean rightGrabberOpen = false;
        long timeBegin, timeCurrent;

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launchServo.setPower(0);
        waitForStart();

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double vertical = gamepad1.left_stick_y;        // Remember, Y stick value is reversed
            double horizontal = gamepad1.left_stick_x;      // Counteract imperfect strafing
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);
            double frontLeftPower = (vertical + horizontal + turn) / denominator;
            double frontRightPower = (vertical - horizontal - turn) / denominator;
            double backLeftPower = (vertical - horizontal + turn) / denominator;
            double backRightPower = (vertical + horizontal - turn) / denominator;

            // robot.setDrivePower(vertical+turn-horizontal,vertical-turn+horizontal,vertical+turn+horizontal,vertical-turn-horizontal);
            // fl, fr, bl, br
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);


            //
            // Game Pad #1
            //

            // launcher
            if (gamepad1.y) {
                launchServo.setPower(1);
                sleep(1000);
                launchServo.setPower(0);
            }

            if (gamepad1.x) {
                grabberL.setPosition(0);
                grabberR.setPosition(0.5);
                RotateArm(1500, 1.0, 2000);
                //sleep(200);
                //encoderDrive(0.2,  -40,  -40, 5.0);
                frontLeftMotor.setPower(0.3);
                frontRightMotor.setPower(0.3);
                backLeftMotor.setPower(0.3);
                backRightMotor.setPower(0.3);

                sleep(2000);
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

                grabberL.setPosition(0.5);
                grabberR.setPosition(0);
                rotator.setPower(0.1);
                sleep(800);

                grabberL.setPosition(0);
                grabberR.setPosition(0.5);

                RotateArm(-1500, 1.0, 4000);

            }

            //
            // Game Pad #2

            // grabber tilt
            if (gamepad2.dpad_down) {       // put down grabbers
                grabberTilt.setPosition(0.1);

            }
            else if (gamepad2.dpad_up) {    // pull up grabbers
                grabberL.setPosition(0);
                grabberR.setPosition(0.5);
                grabberTilt.setPosition(1.0);
            }

            // grabber
            if (gamepad2.left_bumper) {
                //grabberL.setPosition(1);
                //open
                grabberL.setPosition(0.4);
            }
            if (gamepad2.left_trigger > 0.3) {
                //grabberL.setPosition(0.6);
                //close
                grabberL.setPosition(0.2);
            }

            if (gamepad2.right_bumper) {
                //open
                grabberR.setPosition(0);
            }
            if (gamepad2.right_trigger > 0.3) {
                //close
                grabberR.setPosition(0.5);
            }


            // arm rotation

            if (gamepad2.a){
                RotateArm(-200, 1.0, 200);
                rotator.setPower(0.1);
            }
            else if(gamepad2.b){
                RotateArm(200, 1.0, 200);
                rotator.setPower(0.1);
            }

            // actuator extension and retraction
            if (gamepad2.x){
                lifter.setPower(-1.0);
            }
            else if(gamepad2.y){
                lifter.setPower(1.0);
            }
            else {
                lifter.setPower(0);
            }
        }
    }

    void RotateArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(ticks);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(power);
        while(opModeIsActive()
                && rotator.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();
        }
        rotator.setPower(0 * power);
    }
}
