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

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

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

@Autonomous(name=" Auto Drive: Red Near ", group="Robot")
public class Red_Near_Board extends AutoCommon {

    @Override
    public void runOpMode() {

        initAll();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        String line = detectTeamPropLine("red near");
        telemetry.addData("Position: ", line);    //
        while (!isStarted()){
            line = detectTeamPropLine("red near");
            telemetry.update();
        }
        grabberTilt.setPosition(TILT_INIT_POSITION);
        visionPortal.close();
        //line = "left";

        if (line.equals("middle")) {
            encoderDrive(0.2,  30.5,  30.5, 5.0);
            //turnToTargetYaw4(-5+yaw0, 1.0, 6000);
            //encoderDrive(0.2,  26,  26, 5.0);
            //strafe(0.2, 1000);
            dropPixelOnLine();
            encoderDrive(0.3,  -1,  -1, 5.0);
            //strafe(-0.3, 1800);
            strafe_encoder(0.3, 20, 20, 5.0);
            encoderDrive(0.3,  5,  5, 5.0);

            turnToTargetYaw2(90+yaw0, 1.0, 7000);
            //encoderDrive(0.2,  -10,  -10, 5.0);
            RotateArm(RORATE_ARM_TICKS, 1.0, 2000);
            //rotator.setPower(0.1);
            encoderDrive(0.2,  -35,  -35, 5.0);
            grabberL.setPosition(BotCoefficients.grabberLeftOpen);

            sleep(1000);

            RotateArm(-RORATE_ARM_TICKS, 1.0, 2000);
            strafe(0.4,2200);

        }
        else if (line.equals("right")) {
            encoderDrive(0.2,  28,  28, 5.0);
            //strafe(-0.3,700);
            strafe_encoder(0.3, 8, 8, 5.0);
            dropPixelOnLine();
            encoderDrive(0.3,  -2,  -2, 5.0);
            //strafe(-0.3,1500);
            strafe_encoder(0.3, 8, 8, 5.0);
            encoderDrive(0.3,  -2,  -2, 5.0);
            turnToTargetYaw2(90+yaw0, 1, 7000);
            RotateArm(RORATE_ARM_TICKS, 1.0, 2000);
            encoderDrive(0.2,  -35,  -35, 5.0);
            grabberL.setPosition(BotCoefficients.grabberLeftOpen);
            sleep(1000);
            RotateArm(-RORATE_ARM_TICKS, 1.0, 2000);
            strafe(0.4,1800);
            /*
            encoderDrive(0.2,  4,  4, 5.0);
            turnToTargetYaw(-13+yaw0, 0.2, 5000);
            encoderDrive(0.2,  23,  23, 5.0);

            dropPixelOnLine();
            encoderDrive(0.2,  -4,  -4, 5.0);

            turnToTargetYaw4(92+yaw0, 1, 5000);
            encoderDrive(0.2,  -10,  -10, 5.0);

            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.2,  -40,  -40, 5.0);
            grabberL.setPosition(0.4);
            rotator.setPower(0.1);
            sleep(700);

            RotateArm(-RORATE_ARM_TICKS, 1.0, 4000);
            strafe(0.4,2000);

             */
        }
        else {
            encoderDrive(0.3,  28,  28, 5.0);
            //strafe(0.3,1400);
            strafe_encoder(0.3, -20, -20, 5.0);
            dropPixelOnLine();
            //strafe(-0.3,2500);
            strafe_encoder(0.3, 30, 30, 5.0);
            encoderDrive(0.3,  12,  12, 5.0);
            turnToTargetYaw2(90+yaw0, 1, 3000);
            RotateArm(RORATE_ARM_TICKS, 1.0, 2000);
            encoderDrive(0.2,  -40,  -40, 5.0);
            grabberL.setPosition(BotCoefficients.grabberLeftOpen);
            sleep(1000);
            RotateArm(-RORATE_ARM_TICKS, 1.0, 2000);
            strafe(0.4,3200);
            /*
            //driveToRightLine();
            encoderDrive(0.2,  26,  26, 5.0);
            turnToTargetYaw(45+yaw0, 0.2, 5000);
            //encoderDrive(0.2,  38,  38, 5.0);
            //turnToTargetYaw2(-45+yaw0, 0.2, 5000);
            encoderDrive(0.2,  4,  4, 5.0);
            dropPixelOnLine();
            encoderDrive(0.4,  -12,  -12, 5.0);

            turnToTargetYaw4(115+yaw0, 1.0, 8000);
            encoderDrive(0.4,  -10,  -10, 5.0);

            RotateArm(RORATE_ARM_TICKS, 1.0, 4000);
            //rotator.setPower(0.1);
            encoderDrive(0.2,  -40,  -40, 5.0);
            grabberL.setPosition(0.4);
            rotator.setPower(0.1);
            sleep(700);

            RotateArm(-RORATE_ARM_TICKS, 1.0, 4000);
            strafe(-0.4,2000);

             */
        }

        //visionPortal.close();
    }
}
