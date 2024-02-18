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

@Autonomous(name=" Auto Drive: Red Far ", group="Robot")
public class Red_Far_Board extends AutoCommon {

    @Override
    public void runOpMode() {

        initAll();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        grabberTilt.setPosition(0.1);

        String line = detectTeamPropLine("red far");
        //line = "left";
        visionPortal.close();
        if (line.equals("middle")) {
            //driveToMiddleLine();
            encoderDrive(0.2,  31,  31, 5.0);
            //strafe(0.2, 1000);
            dropPixelOnLine();
            encoderDrive(0.4,  -5,  -5, 5.0);
            turnToTargetYaw4(25+yaw0, 0.4, 5000);
            encoderDrive(0.5,   35, 35, 4.0);
            turnToTargetYaw4(90+yaw0, 0.4, 5000);
            encoderDrive(0.5,   -80, -80, 4.0);

            turnToTargetYaw4(55+yaw0, 0.4, 5000);
            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.5,  -60,  -60, 5.0);
            grabberL.setPosition(0.3);
            rotator.setPower(0.1);
            sleep(500);
            //backward(0.2, 5000);
            //dropPixelOnBoard();
        }
        else if (line.equals("left")) {
            //driveToLeftLine();
            //backward(0.2, 2500);
            encoderDrive(0.2,  27,  27, 5.0);
            turnToTargetYaw(30+yaw0, 0.2, 5000);
            //encoderDrive(0.2,  22,  22, 5.0);

            dropPixelOnLine();
            encoderDrive(0.5,  -12,  -12, 5.0);

            turnToTargetYaw4(yaw0, 1.0, 4000);
            //driveToRightLine();
            encoderDrive(0.5,  40,  40, 5.0);
            //turn(0.1, -0.5,1000);
            turnToTargetYaw4(90+yaw0, 1.0, 4000);
            encoderDrive(0.5,  -70,  -70, 5.0);
            turnToTargetYaw4(65+yaw0, 1.0, 4000);

            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.5,  -60,  -60, 5.0);
            grabberL.setPosition(0.3);
            rotator.setPower(0.1);
            sleep(500);
            //driveToLeftLine();

            //turnToTargetYaw(yaw0, 0.4, 5000);
            //encoderDrive(0.2,  -25,  -25, 5.0);
            //turnToTargetYaw(50+yaw0, 0.4, 5000);
            //turn(-0.2, 0.2,800);
            //encoderDrive(0.2,  -10,  -10, 5.0);
            //sleep(100);
            //encoderDrive(0.2,  13,  13, 5.0);
            //turnToTargetYaw(90+yaw0, 0.4, 5000);
            //dropPixelOnLine();
            //forward(0.2, 800);
            //turn(-0.5, 0.3, 1000);
            //driveToBackBoardNearSide(1);
            //backward(0.2, 4500);
            //encoderDrive(0.2,  -48,-48, 5.0);
            //dropPixelOnBoard();
        }
        else {
            //driveToRightLine();
            encoderDrive(0.3,  26,  26, 5.0);
            turnToTargetYaw(-55+yaw0, 0.4, 5000);
            //encoderDrive(0.2,  38,  38, 5.0);
            //turnToTargetYaw2(-45+yaw0, 0.2, 5000);
            encoderDrive(0.5,  2,  2, 5.0);
            dropPixelOnLine();
            encoderDrive(0.5,  -10,  -10, 5.0);
            turnToTargetYaw4(yaw0, 1.0, 5000);
            //driveToRightLine();
            encoderDrive(0.5,  35,  35, 5.0);
            //turn(0.1, -0.5,1000);
            turnToTargetYaw4(90+yaw0, 1.0, 5000);
            encoderDrive(0.5,  -70,  -70, 5.0);
            turnToTargetYaw4(60+yaw0, 1.0, 5000);

            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.5,  -70,  -70, 5.0);
            grabberL.setPosition(0.3);
            rotator.setPower(0.1);
            sleep(500);
        }

        //visionPortal.close();
    }
}
