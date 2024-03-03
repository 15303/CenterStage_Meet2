package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

@TeleOp
public class Teleop extends LinearOpMode {
    private PIDController controller;
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

    public static double p = 0.004, i = 0, d = 0.00003;
    public static double f = 0.005;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;


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



        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launchServo.setPower(0);
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);
            double frontLeftPower = (vertical + horizontal + turn) / denominator;
            double frontRightPower = (vertical - horizontal - turn) / denominator;
            double backLeftPower = (vertical - horizontal + turn) / denominator;
            double backRightPower = (vertical + horizontal - turn) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("pos", rotator.getCurrentPosition());
            telemetry.update();
            //
            // Game Pad #1
            //

            // launcher
            if (gamepad1.y) {
                launchServo.setPower(1);
                sleep(1000);
                launchServo.setPower(0);
            }

            //
            // Game Pad #2

            // grabber tilt
            if (gamepad2.dpad_down) {       // put down grabbers
                // down
                grabberTilt.setPosition(BotCoefficients.grabberDown);
            }
            else if (gamepad2.dpad_up) {    // pull up grabbers
                //up
                grabberTilt.setPosition(BotCoefficients.grabberUp);
            }

            // grabber
            if (gamepad2.left_bumper) {
                // open
                grabberL.setPosition(BotCoefficients.grabberLeftOpen);
            }
            if (gamepad2.left_trigger > 0.3) {
                //close
                grabberL.setPosition(BotCoefficients.grabberLeftClose);
            }

            if (gamepad2.right_bumper) {
                // open
                grabberR.setPosition(BotCoefficients.grabberRightOpen);
            }
            if (gamepad2.right_trigger > 0.3) {
                // close
                grabberR.setPosition(BotCoefficients.grabberRightClose);
            }


            // arm rotation
            if (gamepad2.a){
                target = BotCoefficients.armDown;
            }
            else if(gamepad2.b){
                target = BotCoefficients.armUp;
            }

            controller.setPID(p, i, d);
            int armPos = rotator.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;
            rotator.setPower(power);

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

}
