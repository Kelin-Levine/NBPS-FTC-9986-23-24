package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * Improvements that can be made:
 *
 * Make this into an auto routine interface and make the different routines classes
 * Implement FTCLib motor classes (do this for teleop too)
 * Make everything command based (do this for teleop too)
 * Implement roadrunner
 * Implement that library that optimizes the hardware calls (do this for teleop too)
 */

public class MecanumAutoMode3 extends LinearOpMode {

    // Launcher

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

        // Initialize the robot through the auto routine
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Abort nicely if the code is to be stopped
        if (isStopRequested()) return;

        // Execute the selected auto routine
        switch (selectedRoutine) {
            case NORMAL:
                if (Constants.AUTO_FLIP_CLAWS != null) {
                    runRoutine(isBlueSide, isLongDistance, isDoingAlternateRoute, Constants.AUTO_FLIP_CLAWS, Constants.AUTO_PARK_SIDE);
                } else {
                    runRoutine(isBlueSide, isLongDistance, isDoingAlternateRoute, isBlueSide, Constants.AUTO_PARK_SIDE);
                }
                break;

            case AMBITIOUS:
                runRoutineAmbitious(isBlueSide, isLongDistance, isDoingAlternateRoute, Constants.AUTO_PARK_SIDE);
                break;

            case TESTING:
                runTestingRoutine(gamepad1);
                break;
        }

        // Wait until automatic termination after timer
        while (opModeIsActive()) {
            sleep(50);
        }
    }

    // End of launcher



    // Variables

    // Pre-programmed positions
    private final ArmPosition compactPosition = new ArmPosition(0.18, 0, 1);
    private final ArmPosition spikePlacePosition = new ArmPosition(0.18, 0, 0.8);
    private final ArmPosition onFieldHighPosition = new ArmPosition(0.3, 0, 0.7);
    private final ArmPosition onFieldGrabPosition = new ArmPosition(0.27, 0, 0.7);
    private final ArmPosition backdropPlacePosition = new ArmPosition(0.38, 0, 0.85);

    // Working variables
    final ElapsedTime runtime = new ElapsedTime();
    private Boolean isCameraOpened = null;
    private SidePosition spikeMarkerPosition = SidePosition.UNDEFINED;

    // Hardware objects
    private OpenCvCamera frontCamera;
    private Servo leftClawServo;
    private Servo rightClawServo;
    //private QuadDcMotorArray motors;
    private SampleMecanumDrive mDrive;
    private ArmAssembly armAssembly;


    // Method to initialize the robot
    public void initialize() {
        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        WebcamName frontWebcam = hardwareMap.get(WebcamName.class, "front_webcam");
        int frontCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /*DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");*/

        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        DcMotor armLiftMotor = hardwareMap.get(DcMotor.class, "arm_rotation_motor");
        DcMotor armTravelMotor = hardwareMap.get(DcMotor.class, "arm_extension_motor");
        Servo armWristServo = hardwareMap.get(Servo.class, "arm_wrist_servo");

        // Configure hardware

        // Camera
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(frontWebcam, frontCameraMonitorViewId);
        // The following will run asynchronously. Callback methods are provided to be run when the camera is, or fails to be, opened.
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // This is run if/when the camera is opened successfully.
            @Override
            public void onOpened() {
                // Start camera streaming
                frontCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                isCameraOpened = true;
            }

            // This is run if/when the camera fails to be opened.
            @Override
            public void onError(int errorCode) {
                // Show warning on the driver control station
                isCameraOpened = false;
                telemetry.addData("WARNING: FRONT CAMERA FAILED TO OPEN", "(code " + errorCode + ")");
                telemetry.update();
            }
        });

        // Drive motors
        /*leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // dark voodoo spell to set the p coefficients using evil magical techniques
        ((DcMotorEx) leftFrontDrive).setPositionPIDFCoefficients(Constants.DRIVE_P_COEFFICIENT);
        ((DcMotorEx) leftBackDrive).setPositionPIDFCoefficients(Constants.DRIVE_P_COEFFICIENT);
        ((DcMotorEx) rightFrontDrive).setPositionPIDFCoefficients(Constants.DRIVE_P_COEFFICIENT);
        ((DcMotorEx) rightBackDrive).setPositionPIDFCoefficients(Constants.DRIVE_P_COEFFICIENT);

        motors = new QuadDcMotorArray(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Constants.AUTO_DRIVE_POWER_MULTIPLIER);

        motors.zeroMotorsRunToPosition();*/
        mDrive = new SampleMecanumDrive(hardwareMap);

        // Claw servos
        leftClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setDirection(Servo.Direction.FORWARD);

        leftClawServo.setPosition(Constants.LEFT_CLAW_CLOSED);
        rightClawServo.setPosition(Constants.RIGHT_CLAW_CLOSED);

        // Arm assembly
        armWristServo.setDirection(Servo.Direction.REVERSE);
        armWristServo.setPosition(Constants.WRIST_POSITION_UP);

        armTravelMotor.setDirection(DcMotor.Direction.FORWARD);
        armTravelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(armTravelMotor, Constants.AUTO_ARM_EXTENSION_POWER);

        armLiftMotor.setDirection((DcMotor.Direction.FORWARD));
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(armLiftMotor, Constants.AUTO_ARM_ROTATION_POWER);

        armAssembly = new ArmAssembly(armLiftMotor, armTravelMotor, armWristServo);

        armAssembly.applyPosition(compactPosition);

        // Initialization complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Method to run a version of the routine for testing
    public void runTestingRoutine(Gamepad gamepad1) {
        // START
        telemetry.addData("Starting autonomous testing routine.", "");
        telemetry.addData("", "");
        telemetry.update();

        runtime.reset();

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND SPIKE MARKER
        telemetry.addData("Red spike marker results", "");
        routineFindSpikeMarker(false, false);
        frontCamera.resumeViewport();

        telemetry.addData("", "");
        telemetry.addData("Press south button on gamepad1 to continue.", "");
        telemetry.update();
        while (!gamepad1.a && opModeIsActive()) {
            sleep(50);
        }
        if (isStopRequested()) return;

        telemetry.addData("", "");
        telemetry.addData("Blue spike marker results", "");
        routineFindSpikeMarker(true, false);
        frontCamera.resumeViewport();

        // WAIT FOR INPUT TO PROGRESS
        telemetry.addData("", "");
        telemetry.addData("Vision part 1 (spike markers) complete.", "Press south button on gamepad1 to continue with vision part 2 (apriltag counting).");
        telemetry.update();
        while (!gamepad1.a && opModeIsActive()) {
            sleep(50);
        }
        if (isStopRequested()) return;

        // COUNT APRILTAGS
        testingAprilTagCounter();

        frontCamera.resumeViewport();

        // COMPLETE
        telemetry.addData("Autonomous testing routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }

    // Method to run the normal autonomous routine (50 auto pts. + 3 starting teleop pts.)
    /*
     * Default setup: the purple pixel should be preloaded in the left claw and the yellow
     * pixel should be preloaded in the right claw. This can be swapped.
     * If you mess this up, the 50 point auto will turn into a 10 point auto. Ouch.
     *
     * Reminder: angles in RoadRunnner are measured in counter-clockwise radians.
     */
    public void runRoutine(boolean isBlueSide, boolean isLongDistance, boolean isDoingAlternateRoute, boolean isFlippedClaws, SidePosition parkSide) {
        // START
        telemetry.addData("Starting autonomous routine.", "");
        telemetry.update();

        runtime.reset();

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND SPIKE MARKER
        routineFindSpikeMarker(isBlueSide);

        // PLACE AT SPIKE MARKER
        /*
         * At this point, the robot is still at the starting position.
         */
        routinePlaceAtSpikeMarker(isFlippedClaws);
        mDrive.turn((Math.PI / 2) * (isBlueSide ? 1 : -1));

        // APPROACH BACKDROP
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        routineApproachBackdrop(isBlueSide, isLongDistance, isDoingAlternateRoute);

        // WAIT FOR OTHER ROBOTS
        /*
         * At this point, the robot is centered on the tile in front of the backdrop, facing the
         * backdrop.
         */
        if (isLongDistance) {
            routineWaitForAprilTags();
        }

        // PLACE ON CORRECT SIDE OF BACKDROP
        /*
         * At this point, the robot is still centered on the tile in front of the backdrop, facing
         * the backdrop. placeBackdropPixel() will drive up to and away from the backdrop.
         */
        double sidewaysPlaceOffset = routinePlaceOnBackdrop(isFlippedClaws);

        // PARK
        /*
         * At this point, the robot is about 3 inches away from the front of the backdrop, facing
         * the backdrop. It is to the right (relative) of the center of the backdrop by
         * sidewaysPlaceOffset inches.
         */
        routinePark(isBlueSide, parkSide, sidewaysPlaceOffset);

        // COMPLETE
        telemetry.addData("Autonomous routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }

    // Method to run the ambitious autonomous routine (55 auto pts. + 6 starting teleop pts.)
    /*
     * This routine is best run at the starting position farther from the backdrop.
     * If this routine is run on the red side, the purple pixel should be preloaded in the left claw
     * and the yellow pixel should be preloaded in the right claw.
     * If this routine is run on the blue side, the purple pixel should be preloaded in the right
     * claw and the yellow pixel should be preloaded in the left claw.
     * If you mess this up, the 60 point auto will turn into a 25 point auto.
     */
    public void runRoutineAmbitious(boolean isBlueSide, boolean isLongDistance, boolean isDoingAlternateRoute, SidePosition parkSide) {
        // START
        telemetry.addData("Starting ambitious autonomous routine.", "");
        telemetry.update();

        runtime.reset();

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND SPIKE MARKER
        routineFindSpikeMarker(isBlueSide);

        // PLACE AT SPIKE MARKER
        /*
         * At this point, the robot is still at the starting position.
         */
        routinePlaceAtSpikeMarker(isBlueSide);

        // GRAB ON-FIELD PIXEL
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the opposite alliance. The purple pixel is on the appropriate spike line.
         */
        routineGrabOnFieldPixel(isBlueSide);

        // APPROACH BACKDROP
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        routineApproachBackdrop(isBlueSide, true, isDoingAlternateRoute);

        // WAIT FOR OTHER ROBOTS
        /*
         * At this point, the robot is centered on the tile in front of the backdrop, facing the
         * backdrop.
         */
        routineWaitForAprilTags();

        // PLACE ON CORRECT SIDE OF BACKDROP
        /*
         * At this point, the robot is still centered on the tile in front of the backdrop, facing
         * the backdrop. placeBackdropPixel() will drive up to and away from the backdrop.
         */
        double sidewaysPlaceOffset = routinePlaceDoubleOnBackdrop(isBlueSide);

        // PARK
        /*
         * At this point, the robot is about 3 inches away from the front of the backdrop, facing
         * the backdrop. It is to the right (relative) of the center of the backdrop by
         * sidewaysPlaceOffset inches.
         */
        routinePark(isBlueSide, parkSide, sidewaysPlaceOffset);

        // COMPLETE
        telemetry.addData("Autonomous routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }


    private void routineFindSpikeMarker(boolean isBlueSide) {
        routineFindSpikeMarker(isBlueSide, false);
    }

    private void routineFindSpikeMarker(boolean isBlueSide, boolean isOutputtingChromaChannel) {
        // Find the spike marker
        if (isCameraOpened) {
            // If the camera opened...

            // Resume the front camera's viewport
            frontCamera.resumeViewport();

            // Start the spike vision pipeline
            telemetry.addData("Looking for spike marker...", "");
            telemetry.update();
            VisionPipelineAutoMode2Spike visionPipeline = new VisionPipelineAutoMode2Spike(isBlueSide, isOutputtingChromaChannel);
            frontCamera.setPipeline(visionPipeline);

            // Wait until the pipeline has returned a position
            double startTime = runtime.seconds();
            while (visionPipeline.getSpikePosition() == SidePosition.UNDEFINED) {
                if (runtime.seconds() - startTime > 10) { // timeout after 10 seconds
                    telemetry.addData("Timeout on spike marker vision pipeline!", "(" + (runtime.seconds() - startTime) + " seconds)");
                    break;
                }
                sleep(50);
            }

            // Store determined spike marker position
            if (visionPipeline.getSpikePosition() != SidePosition.UNDEFINED) {
                // If position was found, do so
                spikeMarkerPosition = visionPipeline.getSpikePosition();
                telemetry.addData("Found spike marker at position:", spikeMarkerPosition);
                telemetry.addData("Chroma average 1 (right):", visionPipeline.getChromaAverage1());
                telemetry.addData("Chroma average 2 (middle):", visionPipeline.getChromaAverage2());
                telemetry.addData("Chroma threshold:", Constants.VISION_CHROMA_THRESHOLD);
                telemetry.addData("Difference of chroma averages:", Math.abs(visionPipeline.getChromaAverage1() - visionPipeline.getChromaAverage2()));
                telemetry.update();
            } else {
                // If position was not found, fall back to the fallback spike marker position
                spikeMarkerPosition = Constants.FALLBACK_SPIKE_MARKER_POSITION;
                telemetry.addData("Could not find spike marker.", "Falling back to fallback spike position (" + spikeMarkerPosition + ")");
                telemetry.update();
            }

            // Pause the front camera's viewport to save CPU load
            frontCamera.pauseViewport();

        } else {
            // If the camera failed to open...
            // Fall back to the fallback spike marker position
            spikeMarkerPosition = Constants.FALLBACK_SPIKE_MARKER_POSITION;
            telemetry.addData("Camera failed to open.", "Falling back to fallback spike position (" + spikeMarkerPosition + ")");
            telemetry.update();
        }
    }

    private void routinePlaceAtSpikeMarker(boolean isUsingOppositeClaw) {
        TrajectoryBuilder traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(20, 0), 0), 0);

        Pose2d startingPose = mDrive.getPoseEstimate();

        switch (spikeMarkerPosition) {
            case CENTER:
                // Spike marker is at center position
                mDrive.followTrajectory(traj1
                        .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(4.3, 1.33 * (!isUsingOppositeClaw ? -1 : 1)), 0.3 * (!isUsingOppositeClaw ? -1 : 1)), 0)
                        .build());

                placeSpikePixel(isUsingOppositeClaw);
                break;

            case LEFT:
                // Spike marker is at left position
                mDrive.followTrajectory(traj1
                        .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(0, 0), 0.6), 0)
                        .build());

                placeSpikePixel(isUsingOppositeClaw);
                break;

            case RIGHT:
                // Spike marker is at right position
                mDrive.followTrajectory(traj1
                        .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(0, 0), -0.6), 0)
                        .build());

                placeSpikePixel(isUsingOppositeClaw);
                break;
        }

        /*
         * We assume that a tile is 22.75 inches wide without teeth that the teeth are 0.75 inches
         * wide. We assume that the robot is 18 inches long, so the center of the robot is 9 inches
         * in from the back, so therefore the robot itself needs to be at 7.875 inches out to be
         * centered on the first tile.
         */
        //driveForward(-14.125); // 20 - 14.125 = 5.875
        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(startingPose.getX(), startingPose.getY(), 0), 0)
                .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(-14.125, 0), 0), 0)
                .build();
        mDrive.followTrajectory(traj2);
        // TODO: maybe replace this with a specific position to improve accuracy?
    }

    private void routineGrabOnFieldPixel(boolean isBlueSide) {
        armAssembly.applyPosition(onFieldHighPosition);
        if (isBlueSide) {
            rightClawServo.setPosition(Constants.RIGHT_CLAW_OPEN);
            turnCW90(0.6125);
        } else {
            leftClawServo.setPosition(Constants.LEFT_CLAW_OPEN);
            turnCW90(-0.6125);
        }
        driveForward(22.125);
        armAssembly.applyPosition(onFieldGrabPosition);
        sleep(500);
        if (isBlueSide) {
            rightClawServo.setPosition(Constants.RIGHT_CLAW_CLOSED);
        } else {
            leftClawServo.setPosition(Constants.LEFT_CLAW_CLOSED);
        }
        sleep(500);
        armAssembly.applyPosition(onFieldHighPosition);
        driveForward(-22.125);
        armAssembly.applyPosition(compactPosition);
        turnCW90(isBlueSide ? -1.6 : 1.6);
    }

    private void routineApproachBackdrop(boolean isBlueSide, boolean isLongDistance, boolean isDoingAlternateRoute) {
        // "Premature optimization is the root of all evil" -someone who shouldn't have been ignored
        TrajectoryBuilder traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .splineToSplineHeading(offsetOfCurrentPose(0, (23.5 * (isLongDistance && !isDoingAlternateRoute ? 3 : 1)) * (isBlueSide ? 1 : -1), 0), 0)
                .splineToSplineHeading(offsetOfCurrentPose(23.5 * (isLongDistance && isDoingAlternateRoute ? 2 : 1), 0, 0), 0);

        if (isLongDistance && isDoingAlternateRoute) {
            // Go around the spike marker tile
            traj
                    .splineToSplineHeading(offsetOfCurrentPose(0, 23.5 * 2 * (isBlueSide ? 1 : -1), 0), 0)
                    .splineToSplineHeading(offsetOfCurrentPose(-23.5, 0, 0), 0);
        }

        // Make sure it's still pointing the right direction
        traj
                .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(0, 0), Math.PI / 2 * (isBlueSide ? 1 : -1)), 0);

        // Finally, follow the trajectory
        mDrive.followTrajectory(traj.build());

        armAssembly.applyPosition(backdropPlacePosition);
    }

    private void routineWaitForAprilTags() {
        // Wait until there is nothing in front of the robot
        // Wait until most of the AprilTags on the backdrop are not blocked (2 can be seen)
        if (isCameraOpened) {
            // If the camera opened...

            // Resume the front camera's viewport
            frontCamera.resumeViewport();

            // Start the apriltag vision pipeline
            telemetry.addData("Waiting until AprilTags are not blocked...", "");
            telemetry.update();

            VisionPipelineSimpleAprilTagDetection aprilTagPipeline = new VisionPipelineSimpleAprilTagDetection(
                    Constants.APRILTAG_SIZE,
                    Constants.LENS_FX,
                    Constants.LENS_FY,
                    Constants.LENS_CX,
                    Constants.LENS_CY);
            frontCamera.setPipeline(aprilTagPipeline);

            // Wait until all 3 apriltags have been spotted enough times in a row
            int timesSpottedRepeatedly = 0;
            while (timesSpottedRepeatedly < Constants.VISION_APRILTAG_CHECKS && runtime.seconds() < (30 - Constants.VISION_APRILTAG_TIMEOUT)) {
                sleep(200);

                ArrayList<AprilTagDetection> detections = aprilTagPipeline.getDetectionsUpdate();
                if (detections != null) {
                    if (detections.size() >= 2) {
                        // If we see all 3 tags on the backdrop
                        timesSpottedRepeatedly++;
                    } else {
                        // If we don't see all 3 tags on the backdrop
                        timesSpottedRepeatedly = 0;
                    }
                }
            }

            // Proceed
            if (timesSpottedRepeatedly < Constants.VISION_APRILTAG_CHECKS) {
                telemetry.addData("Running out of time! I hope there's no robot in the way!", "Approaching backdrop immediately.");
            } else {
                sleep(500);
                telemetry.addData("AprilTags are not/no longer blocked. There is not a robot in the way.", "Approaching backdrop.");
            }
            telemetry.update();

            // Pause the front camera's viewport to save CPU load
            frontCamera.pauseViewport();

        } else {
            // If the camera failed to open...
            // Proceed without looking
            telemetry.addData("Camera failed to open. I hope there's no robot in the way!", "Approaching backdrop immediately.");
            telemetry.update();
        }
    }

    private void testingAprilTagCounter() {
        if (isCameraOpened) {
            // If the camera opened...

            // Resume the front camera's viewport
            frontCamera.resumeViewport();

            // Start the apriltag vision pipeline
            telemetry.addData("Counting AprilTags...", "");
            telemetry.update();

            VisionPipelineSimpleAprilTagDetection aprilTagPipeline = new VisionPipelineSimpleAprilTagDetection(
                    Constants.APRILTAG_SIZE,
                    Constants.LENS_FX,
                    Constants.LENS_FY,
                    Constants.LENS_CX,
                    Constants.LENS_CY);
            frontCamera.setPipeline(aprilTagPipeline);

            // Count apriltags numerous times
            for (int i = 0; i < 100; i++) {
                sleep(100);
                telemetry.addData("# of apriltags spotted:", aprilTagPipeline.getLatestDetections().size());
                telemetry.update();
            }

            // Pause the front camera's viewport to save CPU load
            frontCamera.pauseViewport();

        } else {
            // If the camera failed to open...
            // Proceed without looking
            telemetry.addData("Camera failed to open.", "Can't count AprilTags.");
            telemetry.update();
        }
    }

    // Returns: the sideways offset incurred by aligning the pixel
    private double routinePlaceOnBackdrop(boolean isUsingOppositeClaw) {
        double sidewaysPlaceOffset = 0;
        switch (spikeMarkerPosition) {
            case LEFT:
                // Scoring at the left side of the backdrop
                sidewaysPlaceOffset = !isUsingOppositeClaw ? -10.5 : -4;
                break;

            case CENTER:
                // Scoring at the center of the backdrop
                sidewaysPlaceOffset = !isUsingOppositeClaw ? -6 : 6;
                break;

            case RIGHT:
                // Scoring at the right side of the backdrop
                sidewaysPlaceOffset = !isUsingOppositeClaw ? 4 : 10.5;
                break;
        }

        TrajectoryBuilder traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .strafeRight(sidewaysPlaceOffset);
        mDrive.followTrajectory(traj.build());

        placeBackdropPixel(isUsingOppositeClaw, !isUsingOppositeClaw);
        return sidewaysPlaceOffset;
    }

    // Returns: the sideways offset incurred by aligning the pixel
    private double routinePlaceDoubleOnBackdrop(boolean isPrioritizingLeftClaw) {
        /*
         * The pixel in the prioritized claw must be placed at the side corresponding to the spike
         * marker position. The other pixel can go anywhere, as long as it ends up on the backdrop.
         */
        double sidewaysPlaceOffset = 0;
        if (!isPrioritizingLeftClaw) {
            // Place the pixel in the right claw at the corresponding spike marker position
            switch (spikeMarkerPosition) {
                case LEFT:
                    // Scoring at the left side of the backdrop
                    // This has to go twice because there's no way to line up both claws at once
                    driveSideways(-11);
                    placeBackdropPixel(false, true);
                    driveSideways(5);
                    placeBackdropPixel(true, false, 3);
                    sidewaysPlaceOffset = -11 + 5;
                    break;

                case CENTER:
                    // Scoring at the center of the backdrop
                    sidewaysPlaceOffset = -3;
                    driveSideways(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;

                case RIGHT:
                    // Scoring at the right side of the backdrop
                    sidewaysPlaceOffset = 3;
                    driveSideways(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;
            }
        } else {
            // Place the pixel in the left claw at the corresponding spike marker position
            switch (spikeMarkerPosition) {
                case LEFT:
                    // Scoring at the left side of the backdrop
                    sidewaysPlaceOffset = -3;
                    driveSideways(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;

                case CENTER:
                    // Scoring at the center of the backdrop
                    sidewaysPlaceOffset = 3;
                    driveSideways(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;

                case RIGHT:
                    // Scoring at the right side of the backdrop
                    // This has to go twice because there's no way to line up both claws at once
                    driveSideways(11);
                    placeBackdropPixel(true, false);
                    driveSideways(-5);
                    placeBackdropPixel(false, true, 3);
                    sidewaysPlaceOffset = 11 - 5;
                    break;
            }
        }
        return sidewaysPlaceOffset;
    }

    private void routinePark(boolean isBlueSide, SidePosition parkSide, double sidewaysPlaceOffset) {
        // Face away from the driver station so that the heading initializes correctly in teleop
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(offsetOfCurrentPosition(0, 0), 0), 0)
                .build();
        mDrive.followTrajectory(traj1);

        switch (parkSide) {
            case LEFT:
                // Parking at the left side of the backstage
                Trajectory traj2l = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                        .forward((23.5 + sidewaysPlaceOffset) * (isBlueSide ? -1 : 1))
                        .build();
                mDrive.followTrajectory(traj2l);
                break;

            case RIGHT:
                // Parking at the right side of the backstage
                Trajectory traj2r = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                        .forward((23.5 - sidewaysPlaceOffset) * (isBlueSide ? 1 : -1))
                        .build();
                mDrive.followTrajectory(traj2r);
                break;
        }
    }

    // Method to drive forward a specific number of inches
    // This no longer does anything, but is still declared to keep old code from erroring
    private void driveForward(double inches) {
        /*motors.setTargetPosition(Calculations.inchesToEncoderDrive(inches));
        sleepWhileDriveBusy();
        motors.zeroMotorsRunToPosition();*/
    }

    // Method to drive sideways a specific number of inches (to the right)
    // This is not recommended to be used often because it can cause the robot to drift.
    // This no longer does anything, but is still declared to keep old code from erroring
    private void driveSideways(double inches) {
        /*motors.setTargetPosition(Calculations.inchesStrafeToEncodersDrive(inches));
        sleepWhileDriveBusy();
        motors.zeroMotorsRunToPosition();*/
    }

    // Method to turn the robot clockwise in increments of 90 degrees
    // This no longer does anything, but is still declared to keep old code from erroring
    private void turnCW90(double count) {
        /*motors.setTargetPosition(Calculations.clockwise90TurnsToEncodersDrive(count));
        sleepWhileDriveBusy();
        motors.zeroMotorsRunToPosition();*/
    }

    // Method to place a pixel on a spike line
    private void placeSpikePixel(boolean isUsingOppositeClaw) {
        armAssembly.applyPosition(spikePlacePosition);
        sleep(300);
        if (!isUsingOppositeClaw) {
            leftClawServo.setPosition(Constants.LEFT_CLAW_OPEN);
        } else {
            rightClawServo.setPosition(Constants.RIGHT_CLAW_OPEN);
        }
        sleep(200);
        if (!isUsingOppositeClaw) {
            leftClawServo.setPosition(Constants.LEFT_CLAW_CLOSED);
        } else {
            rightClawServo.setPosition(Constants.RIGHT_CLAW_CLOSED);
        }
        armAssembly.applyPosition(compactPosition);
    }

    // Method to approach and place a pixel on the backdrop
    private void placeBackdropPixel(boolean isUsingLeftClaw, boolean isUsingRightClaw) {
        placeBackdropPixel(isUsingLeftClaw, isUsingRightClaw, 14);
    }
    private void placeBackdropPixel(boolean isUsingLeftClaw, boolean isUsingRightClaw, double approachDistance) {
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .forward(approachDistance)
                .build();
        mDrive.followTrajectory(traj1);

        if (isUsingRightClaw) {
            rightClawServo.setPosition(Constants.RIGHT_CLAW_OPEN);
        }
        if (isUsingLeftClaw) {
            leftClawServo.setPosition(Constants.LEFT_CLAW_OPEN);
        }
        sleep(100);

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .back(3)
                .build();
        mDrive.followTrajectory(traj2);

        if (isUsingRightClaw) {
            rightClawServo.setPosition(Constants.RIGHT_CLAW_CLOSED);
        }
        if (isUsingLeftClaw) {
            leftClawServo.setPosition(Constants.LEFT_CLAW_CLOSED);
        }
        armAssembly.applyPosition(compactPosition);
    }

    // Method to generate a pose by offsetting from the current pose
    private Pose2d offsetOfCurrentPose(double dx, double dy, double dh) {
        Pose2d estimate = mDrive.getPoseEstimate();
        return new Pose2d(estimate.getX() + dx, estimate.getY() + dy, estimate.getHeading() + dh);
    }

    // Method to generate a position by offsetting from the current position
    private Vector2d offsetOfCurrentPosition(double dx, double dy) {
        Pose2d estimate = mDrive.getPoseEstimate();
        return new Vector2d(estimate.getX() + dx, estimate.getY() + dy);
    }

    // Method to generate a heading by offsetting from the current heading
    private double offsetOfCurrentHeading(double dh) {
        Pose2d estimate = mDrive.getPoseEstimate();
        return estimate.getHeading() + dh;
    }

    // Method to zero a RUN_TO_POSITION motor
    private void zeroRunToPositionMotor(DcMotor motor, double power) {
        // reset, then power, then position, then mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(power);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Method to sleep until drive motors are done working
    // This no longer does anything, but is still declared to keep old code from erroringA
    private void sleepWhileDriveBusy() {
        /*while (motors.isMotorsBusyFast()) sleep(20);
        sleep(200);*/
    }
}
