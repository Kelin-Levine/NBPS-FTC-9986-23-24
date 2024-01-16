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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * A less rudimentary auto routine
 *
 * Read more information about EasyOpenCV here:
 * https://github.com/OpenFTC/EasyOpenCV/blob/master/readme.md
 */

public class MecanumAutoMode2Launcher extends LinearOpMode {

    public enum Routine {
        NORMAL,
        AMBITIOUS,
        TESTING,
    }


    protected boolean isBlueSide;
    protected boolean isLongDistance;
    protected boolean isDoingAlternateRoute = Constants.AUTO_USE_ALTERNATE_ROUTES;
    protected Routine selectedRoutine = Routine.NORMAL;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        MecanumAutoMode2Routine routine = new MecanumAutoMode2Routine(this, hardwareMap, telemetry);

        // Initialize the robot through the auto routine
        routine.initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Abort nicely if the code is to be stopped
        if (isStopRequested()) return;

        // Execute the selected auto routine
        switch (selectedRoutine) {
            case NORMAL:
                if (Constants.AUTO_FLIP_CLAWS != null) {
                    routine.runRoutine(isBlueSide, isLongDistance, isDoingAlternateRoute, Constants.AUTO_FLIP_CLAWS, Constants.AUTO_PARK_SIDE);
                } else {
                    routine.runRoutine(isBlueSide, isLongDistance, isDoingAlternateRoute, isBlueSide, Constants.AUTO_PARK_SIDE);
                }
                break;

            case AMBITIOUS:
                routine.runRoutineAmbitious(isBlueSide, isLongDistance, isDoingAlternateRoute, Constants.AUTO_PARK_SIDE);
                break;

            case TESTING:
                routine.runTestingRoutine(gamepad1);
                break;
        }

        // Wait until automatic termination after timer
        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
