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
import com.opencsv.CSVReader;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.IOException;
import java.io.FileReader;
import java.util.List;
import java.util.ArrayList;

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

@Autonomous(name=" TEST: Auto Log Replay", group="Robot")
public class AutoLogReplay extends AutoCommon {

    List<LogData> logDataList = new ArrayList<LogData>() ;

    @Override
    public void runOpMode() {

        initAll();

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        loadLogData("/sdcard/FIRST/matchlogs/teleop_log_01.csv");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        //grabberTilt.setPosition(0.2);

        String line = detectTeamPropLine("blue near");
        //line = "middle";
        visionPortal.close();
        replayLogData(line);

        //visionPortal.close();
    }

    public void loadLogData(String fileName) {

        try {
            CSVReader reader = new CSVReader(new FileReader(fileName));
            String[] nextLine;
            String[] lastLine;
            reader.readNext();
            reader.readNext();
            lastLine = reader.readNext();
            //List<LogData> logDataList = new ArrayList<LogData>() ;
            while ((nextLine = reader.readNext()) != null) {
                // nextLine[] is an array of values from the line
                //System.out.println(nextLine[0] + nextLine[1] + "etc...");
                LogData log_item = new LogData();

                double time_last = Double.parseDouble(lastLine[0]);
                double time_next = Double.parseDouble(nextLine[0]);
                log_item.time_interval = (long) ((time_next-time_last)*1000);

                log_item.fl = Double.parseDouble(lastLine[3]);
                log_item.bl = Double.parseDouble(lastLine[4]);
                log_item.fr = Double.parseDouble(lastLine[5]);
                log_item.br = Double.parseDouble(lastLine[6]);
                log_item.rotator = Double.parseDouble(lastLine[7]);
                log_item.lifter = Double.parseDouble(lastLine[8]);
                log_item.launchServo = Double.parseDouble(lastLine[9]);
                log_item.grabberTilt = Double.parseDouble(lastLine[10]);
                log_item.grabberR = Double.parseDouble(lastLine[11]);
                log_item.grabberL = Double.parseDouble(lastLine[12]);
                logDataList.add(log_item);

                lastLine = nextLine;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }


        /*
        try {
            CSVReader reader = new CSVReader(new FileReader(fileName));
            List myEntries = reader.readAll();
        } catch (IOException e) {
        e.printStackTrace();
    }

         */
    }

    public void replayLogData(String line) {
        for (LogData log: logDataList) {
            frontleftDrive.setPower(-log.fl);
            frontrightDrive.setPower(-log.fr);
            backleftDrive.setPower(-log.bl);
            backrightDrive.setPower(-log.br);
            rotator.setPower(log.rotator);
            //lifter.setPower(log.lifter);
            grabberTilt.setPosition(log.grabberTilt);
            grabberR.setPosition(log.grabberR);
            grabberL.setPosition(log.grabberL);
            sleep(85);
            //sleep(log.time_interval);
        }
    }

    private class LogData {
        public long time_interval;
        public double fl;
        public double bl;
        public double fr;
        public double br;
        public double rotator;
        public double lifter;
        public double launchServo;
        public double grabberTilt;
        public double grabberR;
        public double grabberL;
    }
}
