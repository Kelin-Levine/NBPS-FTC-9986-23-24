package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
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

public class MecanumAutoMode3b extends LinearOpMode {

    // Launcher

    public enum Routine {
        NORMAL,
        AMBITIOUS,
    }


    // Protected variables are private variables that are accessible by subclasses and classes in the same "package".
    // Put simply, the package is the folder that the class is in. Classes are grouped by package.
    // These variables are used as settings for what the robot should do when the OpMode is run.
    protected boolean isBlueSide;
    protected boolean isLongDistance;
    protected boolean isDoingAlternateRoute = Constants.AUTO_USE_ALTERNATE_ROUTES;
    protected SidePosition parkSide = Constants.AUTO_PARK_SIDE;
    protected boolean isWaitingForEnd = false;
    protected boolean isMovingInPark = false;
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
                runRoutine(isBlueSide, isLongDistance, isDoingAlternateRoute, isBlueSide, parkSide, isWaitingForEnd, isMovingInPark);
                break;

            case AMBITIOUS:
                runRoutineAmbitious(isBlueSide, isLongDistance, isDoingAlternateRoute, parkSide, isWaitingForEnd, isMovingInPark);
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

    private final double placeInnerOffset = 12.5; // The sideways offset when placing on a side of the backdrop from the side of the claw inner to the backdrop
    private final double placeOuterOffset = 8.5; // The sideways offset when placing on a side of the backdrop from the side of the claw outer to the backdrop
    private final double placeCenterOffset = -4.5; // The sideways offset when placing at the center of the backdrop

    // Working variables
    final ElapsedTime runtime = new ElapsedTime();
    private Boolean isCameraOpened = null;
    private SidePosition spikeMarkerPosition = SidePosition.UNDEFINED;

    // Hardware objects
    private OpenCvCamera frontCamera;
    private SampleMecanumDrive mDrive;
    private ArmSubsystem armSubsystem;


    // Method to initialize the robot
    public void initialize() {
        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        WebcamName frontWebcam = hardwareMap.get(WebcamName.class, "front_webcam");
        int frontCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

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

        // Drivetrain
        mDrive = new SampleMecanumDrive(hardwareMap);

        // Arm assembly
        armSubsystem = new ArmSubsystem(hardwareMap);
        armSubsystem.closeLeftClaw();
        armSubsystem.closeRightClaw();
        armSubsystem.applyPosition(compactPosition);

        // Initialization complete
        telemetry.addData("Status", "Initialized");
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
    public void runRoutine(boolean isBlueSide, boolean isLongDistance, boolean isDoingAlternateRoute, boolean isFlippedClaws, SidePosition parkSide, boolean isWaitingForEnd, boolean isMovingInPark) {
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

        // WAIT FOR OTHER ROBOTS
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        if (isWaitingForEnd) {
            routineWaitAtStart(isLongDistance);
        }

        // APPROACH BACKDROP
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        routineApproachBackdrop(isBlueSide, isLongDistance, isDoingAlternateRoute);

        // PLACE ON CORRECT SIDE OF BACKDROP
        /*
         * At this point, the robot is centered on the tile in front of the backdrop, facing
         * the backdrop. placeBackdropPixel() will drive up to and away from the backdrop.
         */
        double sidewaysPlaceOffset = routinePlaceOnBackdrop(isFlippedClaws);

        // PARK
        /*
         * At this point, the robot is about 3 inches away from the front of the backdrop, facing
         * the backdrop. It is to the right of the center of the backdrop by sidewaysPlaceOffset inches.
         */
        routinePark(isBlueSide, parkSide, sidewaysPlaceOffset);

        // MAKE ROOM FOR OTHER ROBOTS
        /*
         * At this point, the robot has parked at the desired side and is facing forward.
         */
        if (isMovingInPark) {
            routineMoveInPark(isBlueSide);
        }

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
     * If you mess this up, the 55 point auto will turn into a 15 point auto.
     */
    public void runRoutineAmbitious(boolean isBlueSide, boolean isLongDistance, boolean isDoingAlternateRoute, SidePosition parkSide, boolean isWaitingForEnd, boolean isMovingInPark) {
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

        // WAIT FOR OTHER ROBOTS
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        if (isWaitingForEnd) {
            routineWaitAtStart(true);
        }

        // APPROACH BACKDROP
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        routineApproachBackdrop(isBlueSide, true, isDoingAlternateRoute);

        // PLACE ON CORRECT SIDE OF BACKDROP
        /*
         * At this point, the robot is still centered on the tile in front of the backdrop, facing
         * the backdrop. placeBackdropPixel() will drive up to and away from the backdrop.
         */
        double sidewaysPlaceOffset = routinePlaceDoubleOnBackdrop(isBlueSide);

        // PARK
        /*
         * At this point, the robot is about 3 inches away from the front of the backdrop, facing
         * the backdrop. It is to the right of the center of the backdrop by sidewaysPlaceOffset inches.
         */
        routinePark(isBlueSide, parkSide, sidewaysPlaceOffset);

        // MAKE ROOM FOR OTHER ROBOTS
        /*
         * At this point, the robot has parked at the desired side and is facing forward.
         */
        if (isMovingInPark) {
            routineMoveInPark(isBlueSide);
        }

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
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(offsetOfCurrentPosition(19, 0))
                .build();
        mDrive.followTrajectory(traj1);

        Pose2d startingPose = mDrive.getPoseEstimate();

        switch (spikeMarkerPosition) {
            case CENTER:
                // Spike marker is at center position
                mDrive.turn(0.3 * (!isUsingOppositeClaw ? -1 : 1));
                Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                        .lineTo(offsetOfCurrentPosition(4, 1.33 * (!isUsingOppositeClaw ? -1 : 1)))
                        .build();
                mDrive.followTrajectory(traj2);

                placeSpikePixel(isUsingOppositeClaw);

                mDrive.turn(0.3 * (!isUsingOppositeClaw ? 1 : -1));
                break;

            case LEFT:
                // Spike marker is at left position
                mDrive.turn(isBlueSide ? 0.95 : 0.75);
                placeSpikePixel(isUsingOppositeClaw);
                mDrive.turn(isBlueSide ? -0.95 : -0.75);
                break;

            case RIGHT:
                // Spike marker is at right position
                mDrive.turn(isBlueSide ? -0.75 : -0.95);
                placeSpikePixel(isUsingOppositeClaw);
                mDrive.turn(isBlueSide ? 0.75 : 0.95);
                break;
        }

        Trajectory traj3 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(startingPose.getX(), startingPose.getY()))
                .build();
        mDrive.followTrajectory(traj3);

        /*
         * We assume that a tile is 22.75 inches wide without teeth that the teeth are 0.75 inches
         * wide. We assume that the robot is 18 inches long, so the center of the robot is 9 inches
         * in from the back, so therefore the robot itself needs to be at 7.875 inches out to be
         * centered on the first tile.
         */
        Trajectory traj4 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(7.875, 0))
                .build();
        mDrive.followTrajectory(traj4);
    }

    private void routineGrabOnFieldPixel(boolean isBlueSide) {
        armSubsystem.applyPosition(onFieldHighPosition);
        if (isBlueSide) {
            armSubsystem.openRightClaw();
        } else {
            armSubsystem.openLeftClaw();
        }

        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(12.7, 18.1, 0.96 * (isBlueSide ? -1 : 1)))
                .build();
        mDrive.followTrajectory(traj1);

        armSubsystem.applyPosition(onFieldGrabPosition);
        sleep(400);
        if (isBlueSide) {
            armSubsystem.closeRightClaw();
        } else {
            armSubsystem.closeLeftClaw();
        }
        sleep(300);
        armSubsystem.applyPosition(onFieldHighPosition);

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(7.875, 0, (Math.PI / 2) * (isBlueSide ? 1 : -1)))
                .addTemporalMarker(0.5, () -> armSubsystem.applyPosition(compactPosition))
                .build();
        mDrive.followTrajectory(traj2);
    }

    private void routineWaitAtStart(boolean isLongDistance) {
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(23.5 + 7.875, 0))
                .build();
        mDrive.followTrajectory(traj1);

        sleep((int) ((30.0 - runtime.seconds() - (isLongDistance ? Constants.AUTO_MIN_TIME_FAR : Constants.AUTO_MIN_TIME_NEAR)) * 1000));

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(7.875, 0, (Math.PI / 2) * (isBlueSide ? 1 : -1)))
                .build();
        mDrive.followTrajectory(traj2);
    }

    private void routineApproachBackdrop(boolean isBlueSide, boolean isLongDistance, boolean isDoingAlternateRoute) {
        // "Premature optimization is the root of all evil"
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(offsetOfCurrentPosition(0, (23.5 * (isLongDistance && !isDoingAlternateRoute ? 3 : 1)) * (isBlueSide ? 1 : -1)))
                .build();
        mDrive.followTrajectory(traj1);

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(offsetOfCurrentPosition(23.5 * (isLongDistance && isDoingAlternateRoute ? 2 : 1), 0), (Math.PI / 2) * (isBlueSide ? 1 : -1)))
                .build();
        mDrive.followTrajectory(traj2);

        if (isLongDistance && isDoingAlternateRoute) {
            // Go around the spike marker tile
            Trajectory traj3 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineTo(offsetOfCurrentPosition(0, 23.5 * 2 * (isBlueSide ? 1 : -1)))
                    .build();
            mDrive.followTrajectory(traj3);

            Trajectory traj4 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineTo(offsetOfCurrentPosition(-23.5, 0))
                    .build();
            mDrive.followTrajectory(traj4);
        }

        // Make sure it's still pointing the right direction
        /*Trajectory traj5 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(offsetOfCurrentPosition(0, 0), Math.PI / 2 * (isBlueSide ? 1 : -1)))
                .build();
        mDrive.followTrajectory(traj5);*/

        armSubsystem.applyPosition(backdropPlacePosition);
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
        double sidewaysPlaceOffset = 1;
        switch (spikeMarkerPosition) {
            case LEFT:
                // Scoring at the left side of the backdrop
                sidewaysPlaceOffset = !isUsingOppositeClaw ? -placeInnerOffset : -placeOuterOffset;
                break;

            case CENTER:
                // Scoring at the center of the backdrop
                sidewaysPlaceOffset = !isUsingOppositeClaw ? placeCenterOffset : -placeCenterOffset;
                break;

            case RIGHT:
                // Scoring at the right side of the backdrop
                sidewaysPlaceOffset = !isUsingOppositeClaw ? placeOuterOffset : placeInnerOffset;
                break;
        }

        /*TrajectoryBuilder traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate());
        if (sidewaysPlaceOffset > 0) {
            traj.strafeRight(sidewaysPlaceOffset);
        } else {
            traj.strafeLeft(Math.abs(sidewaysPlaceOffset));
        }
        mDrive.followTrajectory(traj.build());*/
        mDrive.strafeRight(sidewaysPlaceOffset);

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
                    mDrive.strafeRight(-placeInnerOffset);
                    placeBackdropPixel(false, true);
                    mDrive.strafeRight(5);
                    placeBackdropPixel(true, false, 3);
                    sidewaysPlaceOffset = -placeInnerOffset + 5;
                    break;

                case CENTER:
                    // Scoring at the center of the backdrop
                    sidewaysPlaceOffset = placeCenterOffset;
                    mDrive.strafeRight(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;

                case RIGHT:
                    // Scoring at the right side of the backdrop
                    sidewaysPlaceOffset = placeOuterOffset;
                    mDrive.strafeRight(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;
            }
        } else {
            // Place the pixel in the left claw at the corresponding spike marker position
            switch (spikeMarkerPosition) {
                case LEFT:
                    // Scoring at the left side of the backdrop
                    sidewaysPlaceOffset = -placeOuterOffset;
                    mDrive.strafeRight(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;

                case CENTER:
                    // Scoring at the center of the backdrop
                    sidewaysPlaceOffset = -placeCenterOffset;
                    mDrive.strafeRight(sidewaysPlaceOffset);
                    placeBackdropPixel(true, true);
                    break;

                case RIGHT:
                    // Scoring at the right side of the backdrop
                    // This has to go twice because there's no way to line up both claws at once
                    mDrive.strafeRight(placeInnerOffset);
                    placeBackdropPixel(true, false);
                    mDrive.strafeRight(-5);
                    placeBackdropPixel(false, true, 3);
                    sidewaysPlaceOffset = placeInnerOffset - 5;
                    break;
            }
        }
        return sidewaysPlaceOffset;
    }

    private void routinePark(boolean isBlueSide, SidePosition parkSide, double sidewaysPlaceOffset) {
        // Face away from the driver station so that the heading initializes correctly in teleop
        mDrive.turn((Math.PI / 2) * (isBlueSide ? -1 : 1));

        // Park on the correct side
        if (parkSide == SidePosition.LEFT
                || (parkSide == SidePosition.OUTSIDE && isBlueSide)
                || (parkSide == SidePosition.INSIDE && !isBlueSide)) {
            // Parking at the left side of the backstage
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .forward((23.5 + sidewaysPlaceOffset) * (isBlueSide ? -1 : 1))
                    .build();
            mDrive.followTrajectory(traj);

        } else if (parkSide == SidePosition.RIGHT
                || (parkSide == SidePosition.OUTSIDE && !isBlueSide)
                || (parkSide == SidePosition.INSIDE && isBlueSide)) {
            // Parking at the right side of the backstage
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .forward((23.5 - sidewaysPlaceOffset) * (isBlueSide ? 1 : -1))
                    .build();
            mDrive.followTrajectory(traj);
        }
    }

    private void routineMoveInPark(boolean isBlueSide) {
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .strafeRight((23.5 * 2) * (isBlueSide ? 1 : -1))
                .build();
        mDrive.followTrajectory(traj1);

        sleep((int) (30000 - runtime.milliseconds() - 1000));

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .strafeLeft((23.5 * 2) * (isBlueSide ? 1 : -1))
                .build();
        mDrive.followTrajectory(traj2);
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
        armSubsystem.applyPosition(spikePlacePosition);
        sleep(300);
        if (!isUsingOppositeClaw) {
            armSubsystem.openLeftClaw();
        } else {
            armSubsystem.openRightClaw();
        }
        sleep(200);
        if (!isUsingOppositeClaw) {
            armSubsystem.closeLeftClaw();
        } else {
            armSubsystem.closeRightClaw();
        }
        armSubsystem.applyPosition(compactPosition);
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
            armSubsystem.openRightClaw();
        }
        if (isUsingLeftClaw) {
            armSubsystem.openLeftClaw();
        }
        sleep(100);

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .back(3)
                .build();
        mDrive.followTrajectory(traj2);

        if (isUsingRightClaw) {
            armSubsystem.closeRightClaw();
        }
        if (isUsingLeftClaw) {
            armSubsystem.closeLeftClaw();
        }
        armSubsystem.applyPosition(compactPosition);
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
