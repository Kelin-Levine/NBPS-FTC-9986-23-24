/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains the NBPS 2023-24 FTC Team 9986's Driver "OpMode".
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * This code applies joystick controls for the claws and four motors on the ground and preset
 * positions for the arm's angle (lift), extension (travel), and wrist.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully retracted.
 *
 * This class works like a main class.
 *
 * This code starts by setting up the robot, then repeats in a loop until the robot is turned off.
 *
 * Robot controls:
 * Controller 1:
 * D-pad up             |   Switch to bottom collection position/increment collection position
 * D-pad down           |   Switch to bottom collection position/decrement collection position
 * D-pad left           |   Close left claw
 * D-pad right          |   Close right claw
 * North (Y/Δ) button   |   Switch to bottom place position/increment place position
 * South (A/X) button   |   Switch to bottom place position/decrement place position
 * West (X/□) button    |   Open left claw
 * East (B/○) button    |   Open right claw
 * Right bumper         |   Move to compact position
 * Left bumper + y button    |   Zero arm lift encoder
 * Left bumper + x button    |   Zero stendo encoder
 *
 * Controller 2:
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   Drive robot (rotate)
 * Left trigger down    |   Decrease driving speed
 * Right trigger down   |   Increase driving speed
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * Above every variable and method, it will say how many times its used. Click on that for a list of exactly where its used.
 */

@TeleOp(name="Gary Mecanum TeleOp Mode 2", group="Linear OpMode")
public class MecanumTeleOpMode2 extends LinearOpMode {

    // Variables
    // The timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Pre-programmed positions
    private final ArmPosition compactPosition = null;//new ArmPosition();
    private final ArmPosition[] collectionPositions = {
            //new ArmPosition(),
    };
    private final ArmPosition[] placePositions = {
            //new ArmPosition(),
    };

    // The set positions that the robot is currently at
    ArmPositionMode armPositionMode = ArmPositionMode.COMPACT;
    int armPositionInSet = 0;


    @Override
    public void runOpMode() {

        boolean driveOnController1 = false;

        // Declare variables that persist between loops
        // The value of inputs at the last loop
        float lTLast = 0.0f;
        float rTLast = 0.0f;
        boolean guideLast = false;


        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        Servo leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        Servo rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        DcMotor armLiftMotor = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        DcMotor armTravelMotor = hardwareMap.get(DcMotor.class, "arm_travel_motor");
        Servo armWristServo = hardwareMap.get(Servo.class, "arm_wrist_servo");

        // Configure motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        QuadMotorArray motors = new QuadMotorArray(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Constants.drivePowerMultiplier);

        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setDirection(Servo.Direction.FORWARD);

        armWristServo.setDirection(Servo.Direction.FORWARD);

        armTravelMotor.setDirection(DcMotor.Direction.FORWARD);
        zeroRunToPositionMotor(armTravelMotor, Constants.armTravelPower);

        armLiftMotor.setDirection((DcMotor.Direction.FORWARD));
        zeroRunToPositionMotor(armLiftMotor, Constants.armLiftPower);

        ArmAssembly armAssembly = new ArmAssembly(armLiftMotor, armTravelMotor, armWristServo);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get current value of inputs now so they can't change during the loop
            // This only matters for inputs that are recorded at the last loop
            float rTNow = driveOnController1 ? gamepad1.right_trigger : gamepad2.right_trigger;
            float lTNow = driveOnController1 ? gamepad1.left_trigger : gamepad2.left_trigger;
            boolean guideNow = gamepad1.guide;

            // Button inputs
            if (gamepad1.left_bumper) {
                // Do zeros here so that they go over normal button inputs
                // Zero the arm lift motor
                if (gamepad1.y) {
                    zeroRunToPositionMotor(armAssembly.getLiftMotor(), Constants.armLiftPower);
                    /*armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armLiftMotor.setPower(0);*/
                }

                // Zero the arm stendo motor
                if (gamepad1.x) {
                    zeroRunToPositionMotor(armAssembly.getTravelMotor(), Constants.armTravelPower);
                }

            } else {
                // Do button inputs here so that they are ignored if both bumpers are pressed
                // Switch between place positions
                if (gamepad1.y) {
                    armAssembly.applyPosition(cycleSetPosition(ArmPositionMode.PLACE, placePositions, 1));
                } else if (gamepad1.a) {
                    armAssembly.applyPosition(cycleSetPosition(ArmPositionMode.PLACE, placePositions, -1));
                }

                // Switch between collection positions
                if (gamepad1.dpad_up) {
                    armAssembly.applyPosition(cycleSetPosition(ArmPositionMode.COLLECT, collectionPositions, 1));
                } else if (gamepad1.dpad_down) {
                    armAssembly.applyPosition(cycleSetPosition(ArmPositionMode.COLLECT, collectionPositions, -1));
                }

                // Apply compact position
                if (gamepad1.right_bumper) {
                    armPositionMode = ArmPositionMode.COMPACT;
                    armPositionInSet = 0;
                    armAssembly.applyPosition(compactPosition);
                }

                // Claws
                if (gamepad1.dpad_left) {
                    leftClawServo.setPosition(1);
                } else if (gamepad1.x) {
                    leftClawServo.setPosition(0);
                }
                if (gamepad1.dpad_right) {
                    rightClawServo.setPosition(1);
                } else if (gamepad1.b) {
                    rightClawServo.setPosition(0);
                }

                // Toggle control setup mode
                if (guideNow && !guideLast) {
                    driveOnController1 = !driveOnController1;
                }
            }


            // Shift the motor power up/down by how much the triggers have been pushed down since the last loop
            // Math.max() picks the greater of two numbers, so negative movements of the trigger (trigger is pulled in) will become 0.0
            // This way, the motor power will only be affected as the triggers are pressed down and not when they are released
            float upShift = Math.max(0.0f, rTNow - rTLast) * 0.05f;
            float downShift = Math.max(0.0f, lTNow - lTLast) * 0.05f;
            motors.setPowerMultiplier(Math.max(Math.min(motors.getPowerMultiplier() + upShift - downShift, 1), 0));

            // Run motors
            float axial = driveOnController1 ? -gamepad1.left_stick_y : -gamepad2.left_stick_y;
            float lateral = driveOnController1 ? gamepad1.left_stick_x : gamepad2.left_stick_x;
            float yaw = driveOnController1 ? gamepad1.right_stick_x : gamepad2.right_stick_x;

            QuadMotorValues drivePower = Calculations.mecanumDrive(axial, lateral, yaw);
            motors.setPower(drivePower);


            // Add info to be shown on the driver control station
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("", "");
            //telemetry.addData("Arm current position:", armLiftPosition);
            //telemetry.addData("Arm target position:", armLiftMotor.getTargetPosition());
            //telemetry.addData("Arm input target:", armInputTarget);
            //telemetry.addData("", "");
            //telemetry.addData("Wrist current position:", armWristServo.get());
            //telemetry.addData("", "");
            telemetry.addData("Overall motor power", (motors.getPowerMultiplier() * 100.0) + "%");
            telemetry.addData("Front left/Right", "%d%%, %d%%", Math.round(drivePower.getLeftFrontValue() * 100), Math.round(drivePower.getRightFrontValue() * 100));
            telemetry.addData("Back  left/Right", "%d%%, %d%%", Math.round(drivePower.getLeftBackValue() * 100), Math.round(drivePower.getRightBackValue() * 100));
            telemetry.update();

            // Update previous inputs
            rTLast = rTNow;
            lTLast = lTNow;
            guideLast = guideNow;
        }
    }

    private void zeroRunToPositionMotor(DcMotor motor, double power) {
        // reset, then power, then position, then mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(power);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private ArmPosition cycleSetPosition(ArmPositionMode mode, ArmPosition[] positions, int increment) {
        if (armPositionMode == mode) {
            armPositionInSet = Math.max(Math.min(armPositionInSet + increment, positions.length-1), 0);
        } else {
            armPositionMode = mode;
            armPositionInSet = 0;
        }
        return positions[armPositionInSet];
    }
}
