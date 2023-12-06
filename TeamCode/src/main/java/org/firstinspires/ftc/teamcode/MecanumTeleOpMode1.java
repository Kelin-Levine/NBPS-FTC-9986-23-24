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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains the NBPS 2023-24 FTC Team 9986's Driver "OpMode".
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * This code drives the four motors on the ground and controls the arm's angle (lift), extension (travel), wrist, and intake.
 *
 * This code starts by setting up the robot, then repeats in a loop until the robot is turned off.
 *
 * Robot controls (one driver mode):
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   (While right bumper NOT held) drive robot (rotate)
 * Right stick          |   (While right bumper IS held) set the arm's target angle
 * Left trigger down    |   Decrease driving speed
 * Right trigger down   |   Increase driving speed
 * X button             |   Extend arm
 * A button             |   Retract arm
 * D-pad up             |   Wrist "Stow" position
 * D-pad left/right     |   Wrist "Place" position
 * D-pad down           |   Wrist "Intake" position
 * Y button             |   Start intake
 * B button             |   Reverse intake
 * Start button         |   Stop intake
 * Left bumper + right bumper + y button    |   Zero arm's encoder
 *
 * In two driver mode, controls are the same on controller 1, except for the following:
 * Left stick (d1)      |   Set the arm's target angle (full range)
 * Right stick (d1)     |   Set the arm's target angle (limited to safe range)
 * Left stick (d2)      |   Drive robot (strafe)
 * Right stick (d2)     |   Drive robot (rotate)
 * Left trigger down (d2)|  Decrease driving speed
 * Right trigger down (d2)| Increase driving speed
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * Above every variable and method, it will say how many times its used. Click on that for a list of exactly where its used.
 */

@TeleOp(name="Gary Mecanum TeleOp Mode 1", group="Linear OpMode")
public class MecanumTeleOpMode1 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        boolean oneDriverMode = false;

        // Declare variables that persist between loops
        // The value of inputs at the last loop
        float lTLast = 0.0f;
        float rTLast = 0.0f;
        boolean guideLast = false;
        boolean intakeStopLast = false;
        // The target position of the arm's lift motor
        int armInputTarget = 0;


        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        CRServo leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        CRServo rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        TouchSensor intakeStopButton = hardwareMap.get(TouchSensor.class, "intake_stop_button");

        Servo armWristServo = hardwareMap.get(Servo.class, "arm_wrist_servo");
        DcMotor armTravelMotor = hardwareMap.get(DcMotor.class, "arm_travel_motor");
        DcMotor armLiftMotor = hardwareMap.get(DcMotor.class, "arm_lift_motor");

        // Configure motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        QuadMotorArray motors = new QuadMotorArray(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Constants.drivePowerMultiplier);

        leftIntakeServo.setDirection(CRServo.Direction.REVERSE);
        rightIntakeServo.setDirection(CRServo.Direction.FORWARD);

        armWristServo.setDirection(Servo.Direction.FORWARD);

        armTravelMotor.setDirection(DcMotor.Direction.FORWARD);
        armTravelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLiftMotor.setDirection((DcMotor.Direction.FORWARD));
        // zero, then power, then position, then mode
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setPower(Constants.armLiftPower);
        armLiftMotor.setTargetPosition(0);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get current value of inputs now so they can't change during the loop
            // This only matters for inputs that are recorded at the last loop
            float rTNow = oneDriverMode ? gamepad1.right_trigger : gamepad2.right_trigger;
            float lTNow = oneDriverMode ? gamepad1.left_trigger : gamepad2.left_trigger;
            boolean guideNow = gamepad1.guide;
            boolean intakeStopNow = intakeStopButton.isPressed();

            // Button inputs
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                // Do zeros here so that they go over normal button inputs
                // Zero the arm motor
                if (gamepad1.y) {
                    armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armLiftMotor.setPower(Constants.armLiftPower);
                    armLiftMotor.setTargetPosition(0);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    /*armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armLiftMotor.setPower(0);*/
                }
            } else {
                // Do button inputs here so that they are ignored if both bumpers are pressed
                // Arm travel motor
                if (gamepad1.x) {
                    armTravelMotor.setPower(Constants.armTravelPower);
                } else if (gamepad1.a) {
                    armTravelMotor.setPower(-Constants.armTravelPower);
                } else {
                    armTravelMotor.setPower(0);
                }

                // Arm wrist motors
                if (gamepad1.dpad_up) {
                    //armWristServo.setPosition(1);
                    armWristServo.setPosition(armWristServo.getPosition() + 0.01);
                    //armWristServo.setPower(-2);
                } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                    //armWristServo.setPosition(-0.5);
                } else if (gamepad1.dpad_down) {
                    //armWristServo.setPosition(0.5);
                    armWristServo.setPosition(armWristServo.getPosition() - 0.01);
                    //armWristServo.setPower(-1);
                }

                // Intake motor
                if (gamepad1.y) {
                    leftIntakeServo.setPower(1);
                    rightIntakeServo.setPower(1);
                } else if (gamepad1.b) {
                    leftIntakeServo.setPower(-1);
                    rightIntakeServo.setPower(-1);
                } else if (gamepad1.start) {
                    leftIntakeServo.setPower(0);
                    rightIntakeServo.setPower(0);
                }

                // Toggle control setup mode
                if (guideNow && !guideLast) {
                    oneDriverMode = !oneDriverMode;
                }
            }

            // Stop intake when pixel hits stop button (there is a button inside the intake that the pixel will hit when it comes in to stop the intake)
            if (intakeStopNow && !intakeStopLast) {
                leftIntakeServo.setPower(0);
                rightIntakeServo.setPower(0);
                // Rumble controller when pixel is intaken
                gamepad1.rumbleBlips(2);
            }


            // Shift the motor power up/down by how much the triggers have been pushed down since the last loop
            // Math.max() picks the greater of two numbers, so negative movements of the trigger (trigger is pulled in) will become 0.0
            // This way, the motor power will only be affected as the triggers are pressed down and not when they are released
            float upShift = Math.max(0.0f, rTNow - rTLast) * 0.05f;
            float downShift = Math.max(0.0f, lTNow - lTLast) * 0.05f;
            motors.setPowerMultiplier(Math.max(Math.min(motors.getPowerMultiplier() + upShift - downShift, 1), 0));

            // Run motors (don't use the right stick if right bumper is held)
            float axial = oneDriverMode ? -gamepad1.left_stick_y : -gamepad2.left_stick_y;
            float lateral = oneDriverMode ? gamepad1.left_stick_x : gamepad2.left_stick_x;
            float yaw = oneDriverMode ? gamepad1.right_stick_x : gamepad2.right_stick_x;

            QuadMotorValues drivePower;
            if (!oneDriverMode || !gamepad1.right_bumper) {
                drivePower = Calculations.mecanumDrive(axial, lateral, yaw);
            } else {
                drivePower = Calculations.mecanumDrive(axial, lateral, 0.0);
            }
            motors.setPower(drivePower);


            // Set the arm's target position
            if (oneDriverMode) {
                // Single controller (use right stick only if right bumper is held)
                double rightStickMagnitude = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
                if (gamepad1.right_bumper && rightStickMagnitude > Constants.armControlStickThreshold) {
                    // have you taken pre-calc yet?
                    //armInputTarget = Math.min(Math.max((int) (Math.atan2(Math.abs(gamepad1.right_stick_x), gamepad1.right_stick_y) / Math.PI * 3400), 0), 3400);
                    if (gamepad1.right_stick_button) {
                        // Range override
                        armInputTarget = Calculations.vectorToArmPositionFull(gamepad1.right_stick_x, gamepad1.right_stick_y);
                    } else {
                        // Normal
                        armInputTarget = Calculations.vectorToArmPositionHalf(gamepad1.right_stick_x, gamepad1.right_stick_y);
                    }
                    armLiftMotor.setTargetPosition(armInputTarget);
                }
            } else {
                // Two controllers (use right stick for half range, left for full)
                double rightStickMagnitude = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
                double leftStickMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                if (rightStickMagnitude > Constants.armControlStickThreshold) {
                    armInputTarget = Calculations.vectorToArmPositionHalf(gamepad1.right_stick_x, gamepad1.right_stick_y);
                    armLiftMotor.setTargetPosition(armInputTarget);
                } else if (leftStickMagnitude > Constants.armControlStickThreshold) {
                    armInputTarget = Calculations.vectorToArmPositionFull(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    armLiftMotor.setTargetPosition(armInputTarget);
                }
            }

            // Light up controller with distance of arm from target position
            int armLiftPosition = armLiftMotor.getCurrentPosition();
            double colorStrength = Math.min(Math.max(Math.abs(armInputTarget - armLiftPosition) - Constants.armColorDeadZone, 0) / Constants.armColorScale, 1);
            gamepad1.setLedColor(0, 1-colorStrength, 0, 100);


            // Add info to be shown on the driver control station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("", "");
            telemetry.addData("Arm current position:", armLiftPosition);
            telemetry.addData("Arm target position:", armLiftMotor.getTargetPosition());
            telemetry.addData("Arm input target:", armInputTarget);
            //telemetry.addData("", "");
            //telemetry.addData("Wrist current position:", armWristServo.get());
            telemetry.addData("", "");
            telemetry.addData("Overall motor power", (motors.getPowerMultiplier() * 100.0) + "%");
            telemetry.addData("Front left/Right", "%d%%, %d%%", Math.round(drivePower.getLeftFrontValue() * 100), Math.round(drivePower.getRightFrontValue() * 100));
            telemetry.addData("Back  left/Right", "%d%%, %d%%", Math.round(drivePower.getLeftBackValue() * 100), Math.round(drivePower.getRightBackValue() * 100));
            telemetry.update();

            // Update previous inputs
            rTLast = rTNow;
            lTLast = lTNow;
            guideLast = guideNow;
            intakeStopLast = intakeStopNow;
        }
    }
}
