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

package org.firstinspires.ftc.teamcode.andyscode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive: Red team away from backboard side", group="Robot")
@Disabled
public class RedAwayFromBoard extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         backleftDrive   = null;

    private DcMotor         backrightDrive  = null;
    private DcMotor         frontleftDrive   = null;

    private DcMotor         frontrightDrive  = null;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    // Tensor flow instance variables
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    //private static final String TFOD_MODEL_ASSET = "model_20230924.tflite";
    private static final String TFOD_MODEL_ASSET = "model_cylinder2_20231125.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
 /*
    private static final String[] LABELS = {
       "pixel yellow", "pixel white", "pixel green", "pixel purple", "Pixel"
    };
*/
    private static final String[] LABELS = {
            "cylinder"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTag;

    // For encoders
    static final double     COUNTS_PER_MOTOR_REV    = 500 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    public Servo pixelServo = null;
    @Override
    public void runOpMode() {

        initTfodAndAprilTag();

        initMotor();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        String line = detectTeamPropLine();


        if (line.equals("middle")) {
            //driveToMiddleLine();
            encoderDrive(0.2,  -34,  -34, 5.0);
            dropPixelOnLine();
            //forward(0.2, 1000);
            encoderDrive(0.2,   20, 20, 4.0);
            //encoderDrive(1,   -5, 5, 4.0);
            //shiftLeft(0.2, 2000);
            turn(0.5, -0.3,1280);
            //encoderDrive(0.5,   , -6, 4.0);
            //driveToBackBoardNearSide(2);
            backward(0.2, 4500);
            dropPixelOnBoard();
        }
        else if (line.equals("left")) {
            //driveToLeftLine();
            //backward(0.2, 2500);
            encoderDrive(0.2,  -15,  -15, 5.0);
            turn(-0.2, 0.2,800);
            encoderDrive(0.2,  -16,  -16, 5.0);
            sleep(1000);
            //dropPixelOnLine();
            //forward(0.2, 800);
            encoderDrive(0.2,  8,  8, 5.0);
            turn(0.2, -0.2, 2000);
            //driveToBackBoardNearSide(1);
            backward(0.2, 2000);
            dropPixelOnBoard();
        }
        else {
            //driveToRightLine();
            encoderDrive(0.2,  -12,  -12, 5.0);
            turn(0.1, -0.5,900);
            encoderDrive(0.2,  -17,  -17, 5.0);
            sleep(1000);
            //dropPixelOnLine();
            //forward(0.2, 2500);
            encoderDrive(0.2,  24,  24, 5.0);
            turn(0.4, -0.1, 2200);
            //driveToBackBoardNearSide(3);
            backward(0.2, 4500);
            dropPixelOnBoard();
        }

        visionPortal.close();
    }
    public void stopRobot(){
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
    }

    public void turn(double powerLeft, double powerRight, long milliseconds){
        frontleftDrive.setPower(powerLeft);
        frontrightDrive.setPower(powerRight);
        backleftDrive.setPower(powerLeft);
        backrightDrive.setPower(powerRight);
        sleep(milliseconds);
        stopRobot();
    }
    public void turnRight(double power, long milliseconds){
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(-power);
        backleftDrive.setPower(power);
        backrightDrive.setPower(-power);
        sleep(milliseconds);
        stopRobot();
    }

    public void forward(double power, int milliseconds) {
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        backleftDrive.setPower(power);
        backrightDrive.setPower(power);

        sleep(milliseconds);
        stopRobot();
    }
    public void backward(double power, int milliseconds){
        frontleftDrive.setPower(-power);
        frontrightDrive.setPower(-power);
        backleftDrive.setPower(-power);
        backrightDrive.setPower(-power);

        sleep(milliseconds);
        stopRobot();
    }
    private void initTfodAndAprilTag() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        builder.addProcessor(aprilTag);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public String detectTeamPropLine(){
        for (int i = 0; i < 200; i++) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.isEmpty()){
                sleep(20);
            }
            else {
                telemetry.addData("# Objects Detected", currentRecognitions.size());

                // Step through the list of recognitions and display info for each one.
                //for (Recognition recognition : currentRecognitions) {
                Recognition recognition = currentRecognitions.get(0);
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                if(x < 150){
                    return "left";
                }
                else{
                    return "middle";
                }
                //}
            }
            // end for() loop
        }
        return "right";
    }
    public void dropPixelOnLine() {
        sleep(100);
    }
    public void dropPixelOnBoard() {

            pixelServo.setPosition(0);
            sleep(1000);
            pixelServo.setPosition(0.1);
            sleep(1000);
            pixelServo.setPosition(0.2);
            sleep(1000);
        pixelServo.setPosition(0.3);
        sleep(1000);
        pixelServo.setPosition(0.4);
        sleep(1000);
            pixelServo.setPosition(1);
            sleep(1000);
            forward(0.2, 300);

    }
    public void driveToBackBoardNearSide(int targetId){
        //adjustment position according to apriltag
        double xPos = 999;
        double yPos = 999;
        int maxTries = 0;
        boolean targetFound = false;
        while (((xPos > 1) || (xPos < -1)) && (maxTries < 10)) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            targetFound = false;
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {

                /*
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }

                 */

                if (detection.id == targetId) {
                    targetFound = true;
                    xPos = detection.ftcPose.x;
                    break;
                }

            }   // end for() loop

            if (targetFound) {

                if (xPos > 1) {
                    shiftLeft(0.2, 500);
                } else if (xPos < -1) {
                    shiftRight(0.2, 500);
                } else {
                    break;
                }
            }
            sleep(20);
            maxTries ++;
        }

        //drive directly to backboard
        backward(0.2, 4500);

    }
    public void initServo() {
        pixelServo = hardwareMap.get(Servo.class, "pixelservo");
        pixelServo.setPosition(1);
    }
    public void initMotor(){
        // Initialize the drive system variables.
        backleftDrive  = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        frontleftDrive  = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backleftDrive.setTargetPosition(newBackLeftTarget);
            backrightDrive.setTargetPosition(newBackRightTarget);
            frontleftDrive.setTargetPosition(newFrontLeftTarget);
            frontrightDrive.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(speed));
            backrightDrive.setPower(Math.abs(speed));
            frontleftDrive.setPower(Math.abs(speed));
            frontrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backleftDrive.isBusy() && backrightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backleftDrive.getCurrentPosition(), backrightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void driveToRightLine(){
        backward(0.2, 2500);
        turnRight(0.5, 500);
    }
    public void driveToMiddleLine() {
        //backward(0.2, 3300);
        encoderDrive(0.2,  -34,  -34, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   24, -24, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, 24, 24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

    }

    public void driveToLeftLine(){
        //backward(0.2, 2500);
        //turnLeft(0.5, 500);
    }




    public void shiftLeft(double power, int milliseconds) {
        frontleftDrive.setPower(-0.2);
        frontrightDrive.setPower(0.2);
        backleftDrive.setPower(0.4);
        backrightDrive.setPower(-0.4);

        sleep(milliseconds);
        stopRobot();
    }
    public void shiftRight(double power, int milliseconds) {
        frontleftDrive.setPower(0.2);
        frontrightDrive.setPower(-0.2);
        backleftDrive.setPower(-0.4);
        backrightDrive.setPower(0.4);

        sleep(milliseconds);
        stopRobot();
    }

}
