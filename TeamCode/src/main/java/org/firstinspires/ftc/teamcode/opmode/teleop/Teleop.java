package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

@TeleOp
public class Teleop extends LinearOpMode {

    private enum State {
        ARM_UP,
        ARM_DOWN,
        HANG_INITIALIZATION,
        HANG,
    }
    State state = State.ARM_DOWN;
    Servo grabberTilt = null;
    Servo grabberR = null;
    Servo grabberL = null;

    // pid

    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.00003;

    public static double f = 0.005;

    public static int target = 0;
    public boolean leftOpen = false;
    public boolean rightOpen = false;
    private final double ticks_in_degree = 700 / 180.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        DcMotor rotator = hardwareMap.dcMotor.get("rotator");
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor lifter = hardwareMap.dcMotor.get("lifter");

        CRServo launchServo = hardwareMap.crservo.get("launcher");
        grabberTilt = hardwareMap.servo.get("grabberTilt");
        grabberR = hardwareMap.servo.get("grabberR");
        grabberL = hardwareMap.servo.get("grabberL");

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backRight.setInverted(true);
        frontRight.setInverted(true);

        controller = new PIDController(p, i, d);

        GamepadEx game1 = new GamepadEx(gamepad1);
        GamepadEx game2 = new GamepadEx(gamepad2);

        launchServo.setPower(0);
        grabberL.setPosition(BotCoefficients.grabberLeftClose);
        grabberR.setPosition(BotCoefficients.grabberRightClose);
        grabberTilt.setPosition(BotCoefficients.grabberUp);

        waitForStart();


        while (!isStopRequested()) {

            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);
            double frontLeftPower = (vertical + horizontal + turn) / denominator;
            double frontRightPower = (vertical - horizontal - turn) / denominator;
            double backLeftPower = (vertical - horizontal + turn) / denominator;
            double backRightPower = (vertical + horizontal - turn) / denominator;

            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);


            if (gamepad1.y) {
                launchServo.setPower(1);
                sleep(1000);
                launchServo.setPower(0);
            }

            if (gamepad2.x){
                // up
                lifter.setPower(-1.0);
            }
            else if(gamepad2.y){
                // down
                lifter.setPower(1.0);
            }
            else {
                lifter.setPower(0);
            }

            if (gamepad2.left_bumper) {
                // open
                grabberL.setPosition(BotCoefficients.grabberLeftOpen);
                leftOpen = true;
            }
            if (gamepad2.left_trigger > 0.3) {
                //close
                grabberL.setPosition(BotCoefficients.grabberLeftClose);
                leftOpen = false;
            }

            if (gamepad2.right_bumper) {
                // open
                grabberR.setPosition(BotCoefficients.grabberRightOpen);
                rightOpen = true;
            }
            if (gamepad2.right_trigger > 0.3) {
                // close
                grabberR.setPosition(BotCoefficients.grabberRightClose);
                rightOpen = false;
            }

            switch (state) {
                case ARM_DOWN:
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

                    if(gamepad2.b){
                        target = BotCoefficients.armUp;
                        state = State.ARM_UP;
                    }
                    if (gamepad1.a){
                        state = State.HANG_INITIALIZATION;
                    }

                    break;
                case ARM_UP:
                    // grabber
                    if (leftOpen && rightOpen){
                        // open
                        sleep(500);
                        grabberL.setPosition(BotCoefficients.grabberLeftClose);
                        sleep(100);
                        grabberR.setPosition(BotCoefficients.grabberRightClose);
                        sleep(500);
                        leftOpen = false;
                        rightOpen = false;
                        target = 10;
                        state = State.ARM_DOWN;
                    }
                    if (gamepad2.a){
                        target = BotCoefficients.armDown;
                        state = State.ARM_DOWN;
                    }
                    break;
                case HANG_INITIALIZATION:
                    target = BotCoefficients.armUp;
                    lifter.setPower(1);
                    sleep(500);
                    while (opModeIsActive()){
                        controller.setPID(p, i, d);
                        int armPos = rotator.getCurrentPosition();
                        double pid = controller.calculate(armPos, target);
                        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                        double power = pid + ff;
                        rotator.setPower(power);

                        if (gamepad1.a){
                            state = State.HANG;
                            break;
                        }
                        vertical = gamepad1.left_stick_y;
                        horizontal = gamepad1.left_stick_x;
                        turn = -gamepad1.right_stick_x;

                        denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);
                        frontLeftPower = (vertical + horizontal + turn) / denominator;
                        frontRightPower = (vertical - horizontal - turn) / denominator;
                        backLeftPower = (vertical - horizontal + turn) / denominator;
                        backRightPower = (vertical + horizontal - turn) / denominator;

                        frontLeft.set(frontLeftPower);
                        frontRight.set(frontRightPower);
                        backLeft.set(backLeftPower);
                        backRight.set(backRightPower);
                    }

                case HANG:
                    target = 10;
                    lifter.setPower(-1);
                    sleep(3000);
                    while (opModeIsActive()){
                        controller.setPID(p, i, d);
                        int armPos = rotator.getCurrentPosition();
                        double pid = controller.calculate(armPos, target);
                        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                        double power = pid + ff;
                        rotator.setPower(power);
                    }


            }

            controller.setPID(p, i, d);
            int armPos = rotator.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;
            rotator.setPower(power);

        }

    }






}