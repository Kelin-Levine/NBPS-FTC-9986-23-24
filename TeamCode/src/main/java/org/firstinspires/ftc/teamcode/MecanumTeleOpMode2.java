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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains the NBPS 2023-24 FTC Team 9986's Driver "OpMode".
 * An OpMode (Operation Mode) is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The OpMode to use can be selected on the FTC Driver Station.
 *
 * This code applies joystick controls for the claws and four motors on the ground and preset
 * positions for the arm's angle (lift), extension (travel), and wrist. The driving is field-centric,
 * unless toggled off.
 *
 * Before starting this OpMode, the arm lift must be in its lowest position and the extension fully
 * retracted. The robot should also be facing forward, or else the field-centric driving will be off until reset.
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
 * North (Y/Δ) button   |   Switch to bottom place position/increment place position (also sets drive to slow speed)
 * South (A/X) button   |   Switch to bottom place position/decrement place position (also sets drive to slow speed)
 * West (X/□) button    |   Open left claw
 * East (B/○) button    |   Open right claw
 * Right bumper         |   Move to compact position
 * Left bumper + y button    |   Zero arm lift encoder
 * Left bumper + x button    |   Zero stendo encoder
 * Left stick up/down   |   Raise/lower arm lift manually *DOES NOT WORK
 * Right stick up/down  |   Extend/retract arm wrist manually *DOES NOT WORK
 * Select button        |   Engage/toggle hang positions
 * Start button         |   Toggle showing extra robot information on the Driver Station
 *
 * Controller 2:
 * Left stick           |   Drive robot (strafe)
 * Right stick          |   Drive robot (rotate)
 * Left bumper          |   Set to slow drive speed
 * Right bumper         |   Set to fast drive speed
 * Left trigger down    |   Decrease drive speed
 * Right trigger down   |   Increase drive speed
 * North (Y/Δ) button   |   Zero robot heading
 * Guide button         |   Toggle field-centric driving
 * Select button        |   Launch drone
 * Start button         |   Toggle showing extra robot information on the Driver Station
 *
 * Tips:
 * If you don't understand what a particular method does, then hover your mouse over it to get some info and a short description.
 * Above every variable and method, it will say how many times its used. Click on that for a list of exactly where its used.
 */

@Disabled
@TeleOp(name="Gary Mecanum TeleOp Mode 2", group="Linear OpMode")
public class MecanumTeleOpMode2 extends LinearOpMode {

    // Enums to label the different position modes that the arm can be in
    public enum ArmPositionMode {
        COMPACT,
        COLLECT,
        PLACE,
        HANG,
        MANUAL,
    }


    // Variables
    // The timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Pre-programmed positions
    private final ArmPosition compactPosition = new ArmPosition(0.18, 0, 1);
    private final ArmPosition[] collectionPositions = {
            new ArmPosition(0.18, 0, 0.72),
            new ArmPosition(0.231, 0, 0.72),
            //new ArmPosition(0.18, 0.75, 0.655),
            //new ArmPosition(0.231, 0.75, 0.65),
    };
    private final ArmPosition[] placePositions = {
            new ArmPosition(0.5, 0, 0.8),
            new ArmPosition(0.534, 0, 0.79),
            new ArmPosition(0.6, 0, 0.78),
    };
    private final ArmPosition[] hangPositions = {
            new ArmPosition(0.9, 0, 0.4),
            new ArmPosition(1.95, 0, 0.4),
    };

    // The set positions that the robot is currently at
    ArmPositionMode armPositionMode = ArmPositionMode.COMPACT;
    int armPositionInSet = 0;


    @Override
    public void runOpMode() {

        boolean isShowingExtraInfo = false;
        boolean isFieldCentric = true;

        // Declare variables that persist between loops
        // The value of inputs at the last loop
        boolean aButton1Last = false;
        boolean xButton1Last = false;
        boolean yButton1Last = false;
        boolean dPadUp1Last = false;
        boolean dPadDown1Last = false;
        boolean selectButton1Last = false;
        float leftTrigger2Last = 0.0f;
        float rightTrigger2Last = 0.0f;
        boolean yButton2Last = false;
        boolean guideButton2Last = false;
        boolean startButtonLast = false;


        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        Servo leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        Servo rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        Servo droneReleaseServo = hardwareMap.get(Servo.class, "drone_release_servo");

        DcMotor armLiftMotor = hardwareMap.get(DcMotor.class, "arm_rotation_motor");
        DcMotor armTravelMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        Servo armWristServo = hardwareMap.get(Servo.class, "arm_wrist_servo");

        // Configure hardware
        // Drive motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        QuadDcMotorArray motors = new QuadDcMotorArray(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Constants.DRIVE_POWER_MULTIPLIER);

        // Claw servos
        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setDirection(Servo.Direction.FORWARD);

        // Drone release servo
        droneReleaseServo.setDirection(Servo.Direction.FORWARD);
        droneReleaseServo.setPosition(Constants.DRONE_RELEASE_CLOSED);

        // Arm assembly
        armWristServo.setDirection(Servo.Direction.REVERSE);

        armTravelMotor.setDirection(DcMotor.Direction.FORWARD);
        armTravelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(armTravelMotor, Constants.ARM_EXTENSION_POWER);

        armLiftMotor.setDirection((DcMotor.Direction.FORWARD));
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(armLiftMotor, Constants.ARM_ROTATION_POWER);

        ArmAssembly armAssembly = new ArmAssembly(armLiftMotor, armTravelMotor, armWristServo);

        // Initialize arm for play
        armPositionMode = ArmPositionMode.COMPACT;
        armPositionInSet = 0;
        armAssembly.applyPosition(compactPosition);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu1");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get current value of inputs now so they can't change during the loop
            // This only matters for inputs that are recorded at the last loop
            boolean aButton1Now = gamepad1.a;
            boolean xButton1Now = gamepad1.x;
            boolean yButton1Now = gamepad1.y;
            boolean dPadUp1Now = gamepad1.dpad_up;
            boolean dPadDown1Now = gamepad1.dpad_down;
            boolean selectButton1Now = gamepad1.share;
            float rightTrigger2Now = gamepad2.right_trigger;
            float leftTrigger2Now = gamepad2.left_trigger;
            boolean yButton2Now = gamepad2.y;
            boolean guideButton2Now = gamepad2.guide;
            boolean startButtonNow = gamepad1.start || gamepad2.start;

            // Button inputs
            if (gamepad1.left_bumper) {
                // Do zeros here so that they go over normal button inputs
                // Zero the arm lift motor
                if (yButton1Now && !yButton1Last) {
                    zeroRunToPositionMotor(armAssembly.getLiftMotor(), Constants.ARM_ROTATION_POWER);
                }

                // Zero the arm stendo motor
                if (xButton1Now && !xButton1Last) {
                    zeroRunToPositionMotor(armAssembly.getTravelMotor(), Constants.ARM_EXTENSION_POWER);
                }

            } else {
                // Do button inputs here so that they are ignored if both bumpers are pressed
                // Switch between place positions
                if (yButton1Now && !yButton1Last) {
                    ArmPosition position = cycleSetPosition(ArmPositionMode.PLACE, placePositions, 1);
                    armAssembly.applyPosition(position);
                } else if (aButton1Now && !aButton1Last) {
                    ArmPosition position = cycleSetPosition(ArmPositionMode.PLACE, placePositions, -1);
                    armAssembly.applyPosition(position);
                }

                // Switch between collection positions
                if (dPadUp1Now && !dPadUp1Last) {
                    ArmPosition position = cycleSetPosition(ArmPositionMode.COLLECT, collectionPositions, 1);
                    armAssembly.applyPosition(position);
                } else if (dPadDown1Now && !dPadDown1Last) {
                    ArmPosition position = cycleSetPosition(ArmPositionMode.COLLECT, collectionPositions, -1);
                    armAssembly.applyPosition(position);
                }

                // Apply compact position
                if (gamepad1.right_bumper) {
                    armPositionMode = ArmPositionMode.COMPACT;
                    armPositionInSet = 0;
                    armAssembly.applyPosition(compactPosition);
                }

                // Do hang positions
                if (selectButton1Now && !selectButton1Last) {
                    // always start at first hang position, do not increment unless already at a hang position
                    ArmPosition position = cycleSetPosition(ArmPositionMode.HANG, hangPositions, armPositionMode == ArmPositionMode.HANG ? 1 : 0);
                    armAssembly.applyPosition(position);
                }

                // Claws
                if (gamepad1.dpad_left) {
                    leftClawServo.setPosition(Constants.LEFT_CLAW_CLOSED);
                } else if (gamepad1.x) {
                    leftClawServo.setPosition(Constants.LEFT_CLAW_OPEN);
                }
                if (gamepad1.dpad_right) {
                    rightClawServo.setPosition(Constants.RIGHT_CLAW_CLOSED);
                } else if (gamepad1.b) {
                    rightClawServo.setPosition(Constants.RIGHT_CLAW_OPEN);
                }

                // Drone release
                if (gamepad2.share) {
                    droneReleaseServo.setPosition(Constants.DRONE_RELEASE_OPEN);
                }

                // Set to slow drive speed
                if (gamepad2.left_bumper || (yButton1Now && !yButton1Last) || (aButton1Now && !aButton1Last)) {
                    motors.setPowerMultiplier(0.4);
                }

                // Set to fast drive speed
                if (gamepad2.right_bumper) {
                    motors.setPowerMultiplier(0.9);
                }

                // Toggle field-centric
                if (guideButton2Now && !guideButton2Last) {
                    isFieldCentric = !isFieldCentric;
                }

                // Zero heading
                if (yButton2Now && !yButton2Last) {
                    imu.resetYaw();
                }

                // Toggle extra robot information
                if (startButtonNow && !startButtonLast) {
                    isShowingExtraInfo = !isShowingExtraInfo;
                }
            }


            // Offset the wrist's reference position up/down by how much the triggers have been pushed down since the last loop
            // Math.max() picks the greater of two numbers, so negative movements of the trigger (trigger is pulled in) will become 0.0
            // This way, the wrist will only be affected as the triggers are pressed down and not when they are released
            float upShift = Math.max(0.0f, rightTrigger2Now - rightTrigger2Last) * 0.1f;
            float downShift = Math.max(0.0f, leftTrigger2Now - leftTrigger2Last) * 0.1f;
            motors.setPowerMultiplier(Math.max(Math.min(motors.getPowerMultiplier() + upShift - downShift, 1), 0));

            /*double upShift = Math.max(0.0f, leftTrigger2Now - leftTrigger2Last) * 0.1;
            double downShift = Math.max(0.0f, rightTrigger2Now - rightTrigger2Last) * 0.1;
            Constants.WRIST_POSITION_TOP = Math.max(Math.min(Constants.WRIST_POSITION_TOP + upShift - downShift, 1), 0);
            armAssembly.applyWristPosition(armAssembly.getWristTargetPositionScaled()); // Refresh wrist position to use new reference position*/

            // Run motors
            float axial = -gamepad2.left_stick_y;
            float lateral = gamepad2.left_stick_x;
            float yaw = gamepad2.right_stick_x;
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            QuadMotorValues<Double> drivePower;
            if (isFieldCentric) {
                drivePower = Calculations.mecanumDriveisFieldCentric(axial, lateral, yaw, heading);
            } else {
                drivePower = Calculations.mecanumDriveRobotCentric(axial, lateral, yaw);
            }
            motors.setPower(drivePower);

            // Manual control of arm lift and wrist (this is a redundancy so it's ok that it's a little messy)
            /*if (gamepad1.left_stick_y > 0.2) {
                armPositionMode = ArmPositionMode.MANUAL;
                armPositionInSet = 0;
                armAssembly.applyLiftPosition(Calculations.encoderToScaleArmLift(armLiftMotor.getCurrentPosition()) + (gamepad1.left_stick_y + (gamepad1.left_stick_y > 0 ? -0.25 : 0.25)));
            }
            // Wrist
            if (gamepad1.right_stick_y > 0.2) {
                armPositionMode = ArmPositionMode.MANUAL;
                armPositionInSet = 0;
                armAssembly.applyWristPosition(Calculations.encoderToScaleArmWrist(armWristServo.getPosition()) + (gamepad1.right_stick_y + (gamepad1.right_stick_y > 0 ? -0.25 : 0.25)));
            }*/


            // Add info to be shown on the driver control station
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("", "");
            telemetry.addData("Arm mode:", armPositionMode);
            telemetry.addData("", "");
            telemetry.addData("Field-centric driving:", isFieldCentric);
            telemetry.addData("Heading direction (degrees):", (heading / Math.PI * 180));
            telemetry.addData("", "");
            if (isShowingExtraInfo) {
                // This is the only part that could be sped up with bulk reads. Seeing as it's not
                // essential, bulk reads would only introduce additional overhead compared to a switch like this.
                telemetry.addData("Additional info:", "");
                telemetry.addData("", "");
                telemetry.addData("Left  claw servo position:", leftClawServo.getPosition());
                telemetry.addData("Right claw servo position:", rightClawServo.getPosition());
                telemetry.addData("", "");
                telemetry.addData("Wrist scaled position:", armAssembly.getWristTargetPositionScaled());
                telemetry.addData("Wrist servo position:", armWristServo.getPosition());
                telemetry.addData("", "");
                telemetry.addData("Arm stendo scaled position:", armAssembly.getTravelTargetPositionScaled());
                telemetry.addData("Arm stendo encoder position:", armTravelMotor.getCurrentPosition());
                telemetry.addData("Arm lift target position:", armLiftMotor.getTargetPosition());
                telemetry.addData("", "");
                telemetry.addData("Arm lift scaled position:", armAssembly.getLiftTargetPositionScaled());
                telemetry.addData("Arm lift encoder position:", armLiftMotor.getCurrentPosition());
                telemetry.addData("Arm lift target position:", armLiftMotor.getTargetPosition());
                telemetry.addData("", "");
                telemetry.addData("Drive motor power level:", (motors.getPowerMultiplier() * 100.0) + "%");
                telemetry.addData("Front left/right encoder position:", "%d, %d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.addData("Back  left/right encoder position:", "%d, %d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addData("Front left/right motor power:", "%d%%, %d%%", Math.round(leftFrontDrive.getPower() * 100), Math.round(rightFrontDrive.getPower() * 100));
                telemetry.addData("Back  left/right motor power:", "%d%%, %d%%", Math.round(leftBackDrive.getPower() * 100), Math.round(rightBackDrive.getPower() * 100));
            } else {
                telemetry.addData("Field-centric driving:", isFieldCentric);
                telemetry.addData("Heading direction (degrees):", (heading / Math.PI * 180));
                telemetry.addData("Drive motor power level:", (motors.getPowerMultiplier() * 100.0) + "%");
                telemetry.addData("", "");
                telemetry.addData("Robot info hidden for performance.", "Press start to toggle.");
            }
            telemetry.update();

            // Update previous inputs
            aButton1Last = aButton1Now;
            xButton1Last = xButton1Now;
            yButton1Last = yButton1Now;
            dPadUp1Last = dPadUp1Now;
            dPadDown1Last = dPadDown1Now;
            selectButton1Last = selectButton1Now;
            rightTrigger2Last = rightTrigger2Now;
            leftTrigger2Last = leftTrigger2Now;
            yButton2Last = yButton2Now;
            guideButton2Last = guideButton2Now;
            startButtonLast = startButtonNow;
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
        if (armPositionMode != mode) {
            armPositionMode = mode;
            armPositionInSet = 0;
        }
        armPositionInSet = Math.max(Math.min(armPositionInSet + increment, positions.length-1), 0);
        return positions[armPositionInSet];
    }
}
