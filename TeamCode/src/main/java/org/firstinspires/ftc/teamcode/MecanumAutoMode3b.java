package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    // These variables can be changed by a launcher class to start the code with different settings. See automode3blauncher
    protected boolean isBlueSide = false;
    protected boolean isLongDistance = false;
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
                runRoutine();
                break;

            case AMBITIOUS:
                runRoutineAmbitious();
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
    private final ArmPosition onFieldHighPosition = new ArmPosition(0.5, 0, 0.3);
    private final ArmPosition onFieldGrabPosition = new ArmPosition(0.28, 0, 0.74);
    private final ArmPosition backdropPlacePosition =new ArmPosition(0.44, 0, 0.88);

    private final double placeOuterOffset = 11.0; // The sideways offset when placing on a side of the backdrop from the side of the claw outer to the side of the backdrop
    private final double placeInnerOffset = 5.0; // The sideways offset when placing on a side of the backdrop from the side of the claw inner to the backdrop
    private final double placeCenterOffset = -2.5; // The sideways offset when placing at the center of the backdrop

    // Working variables
    final ElapsedTime runtime = new ElapsedTime();
    private Boolean isCameraOpened = null;
    private SidePosition spikeMarkerPosition = SidePosition.UNDEFINED;
    private double faceBackdropAngle;

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
        armSubsystem.applyWristPosition(1.1);

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
    public void runRoutine() {
        // START
        telemetry.addData("Starting autonomous routine.", "");
        telemetry.addData("", "");
        telemetry.update();

        runtime.reset();

        faceBackdropAngle = (Math.PI / 2) * (isBlueSide ? 1 : -1);

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND SPIKE MARKER
        routineFindSpikeMarker();

        // PLACE AT SPIKE MARKER
        /*
         * At this point, the robot is still at the starting position.
         */
        routinePlaceAtSpikeMarker(isBlueSide, faceBackdropAngle);
        //mDrive.turnTo(faceBackdropAngle);

        telemetry.addData("Time spent spike markering", runtime.time());
        telemetry.update();

        // WAIT FOR OTHER ROBOTS
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        if (isWaitingForEnd) {
            routineWaitAtStart(isLongDistance, isDoingAlternateRoute, false);
        }

        // APPROACH BACKDROP
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        double startScoringTime = runtime.time();

        routineApproachBackdrop(isLongDistance, isDoingAlternateRoute);

        // PLACE ON CORRECT SIDE OF BACKDROP
        /*
         * At this point, the robot is centered on a tile to the side of the tile in front of the
         * backdrop, facing the backdrop. placeBackdropPixel() will drive up to and away from the backdrop.
         */
        routinePlaceOnBackdrop(isBlueSide);

        telemetry.addData("Time spent scoring", runtime.time() - startScoringTime);
        telemetry.update();

        // PARK
        /*
         * At this point, the robot is about 6 inches away from the front of the backdrop, facing
         * the backdrop. It is offset from the center of the backdrop by some distance.
         */
        routinePark(parkSide);

        telemetry.addData("Time spent scoring + parking", runtime.time() - startScoringTime);
        telemetry.update();

        // MAKE ROOM FOR OTHER ROBOTS
        /*
         * At this point, the robot has parked at the desired side and is facing forward.
         */
        if (isMovingInPark) {
            routineMoveInPark();
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
    public void runRoutineAmbitious() {
        // START
        telemetry.addData("Starting ambitious autonomous routine.", "");
        telemetry.update();

        runtime.reset();

        faceBackdropAngle = (Math.PI / 2) * (isBlueSide ? 1 : -1);

        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // FIND SPIKE MARKER
        routineFindSpikeMarker();

        // PLACE AT SPIKE MARKER
        /*
         * At this point, the robot is still at the starting position.
         */
        double pickupAngle = 1.0 * (isBlueSide ? -1 : 1);
        routinePlaceAtSpikeMarker(isBlueSide, pickupAngle);

        // GRAB ON-FIELD PIXEL
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the opposite alliance. The purple pixel is on the appropriate spike line.
         */
        routineGrabOnFieldPixel(pickupAngle);

        telemetry.addData("Time spent spike markering", runtime.time());
        telemetry.update();

        // WAIT FOR OTHER ROBOTS
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        if (isWaitingForEnd) {
            routineWaitAtStart(true, isDoingAlternateRoute, true);
        }

        // APPROACH BACKDROP
        /*
         * At this point, the robot is centered on the tile that it started on and is
         * facing the backdrop.
         */
        double startScoringTime = runtime.time();

        routineApproachBackdrop(true, isDoingAlternateRoute);

        // PLACE ON CORRECT SIDE OF BACKDROP
        /*
         * At this point, the robot is centered on a tile to the side of the tile in front of the
         * backdrop, facing the backdrop. placeBackdropPixel() will drive up to and away from the backdrop.
         */
        routinePlaceDoubleOnBackdrop(isBlueSide);

        telemetry.addData("Time spent scoring double", runtime.time() - startScoringTime);
        telemetry.update();

        // PARK
        /*
         * At this point, the robot is about 6 inches away from the front of the backdrop, facing
         * the backdrop. It is offset from the center of the backdrop by some distance.
         */
        routinePark(parkSide);

        telemetry.addData("Time spent scoring + parking", runtime.time() - startScoringTime);
        telemetry.update();

        // MAKE ROOM FOR OTHER ROBOTS
        /*
         * At this point, the robot has parked at the desired side and is facing forward.
         */
        if (isMovingInPark) {
            routineMoveInPark();
        }

        // COMPLETE
        telemetry.addData("Autonomous routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }


    private void routineFindSpikeMarker() {
        routineFindSpikeMarker(false);
    }

    private void routineFindSpikeMarker(boolean isOutputtingChromaChannel) {
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
        telemetry.addData("", "");
        telemetry.update();
    }

    private void routinePlaceAtSpikeMarker(boolean isUsingRightClaw, double finalHeading) {
        boolean isTrussOnLeft = (isBlueSide && isLongDistance) || !(isBlueSide || isLongDistance);

        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(15, 3 * (isTrussOnLeft ? -1 : 1)))
                .build();
        mDrive.followTrajectory(traj1);

        Pose2d startingPose = mDrive.getPoseEstimate();

        switch (spikeMarkerPosition) {
            case CENTER:
                // Spike marker is at center position
                mDrive.turnTo(0.5 * (!isUsingRightClaw ? -1 : 1));
                Trajectory traj2c = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                        .lineTo(new Vector2d(24.0, 0.0 * (!isUsingRightClaw ? -1 : 1)))
                        .build();
                mDrive.followTrajectory(traj2c);

                placeSpikePixel1();
                placeSpikePixel2(isUsingRightClaw);

                //mDrive.turnTo(0);
                break;

            case LEFT:
                // Spike marker is at left position
                if (isTrussOnLeft) {
                    // Navigate around truss
                    mDrive.turnTo(0.8);

                    placeSpikePixel1();

                    Trajectory traj2l = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                            .forward(9)
                            .build();
                    mDrive.followTrajectory(traj2l);

                    placeSpikePixel2(isUsingRightClaw);
                } else {
                    // Drop down and slide claw across to move marker out of the way
                    mDrive.turnTo(isUsingRightClaw ? 1.1 : 0.75);

                    placeSpikePixel1();

                    Trajectory traj2l = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                            .lineTo(new Vector2d(22, (isUsingRightClaw ? 3 : 0)))
                            .build();
                    mDrive.followTrajectory(traj2l);

                    placeSpikePixel2(isUsingRightClaw);
                }

                //mDrive.turnTo(0);
                break;

            case RIGHT:
                // Spike marker is at right position
                if (!isTrussOnLeft) {
                    // Navigate around truss
                    mDrive.turnTo(-0.8);

                    placeSpikePixel1();

                    Trajectory traj2r = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                            .forward(9)
                            .build();
                    mDrive.followTrajectory(traj2r);

                    placeSpikePixel2(isUsingRightClaw);
                } else {
                    // Drop down and slide claw across to move marker out of the way
                    mDrive.turnTo(isUsingRightClaw ? -0.75 : -1.1);

                    placeSpikePixel1();

                    Trajectory traj2r = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                            .lineTo(new Vector2d(22, (!isUsingRightClaw ? -3 : 0)))
                            .build();
                    mDrive.followTrajectory(traj2r);

                    placeSpikePixel2(isUsingRightClaw);
                }

                //mDrive.turnTo(0);
                break;
        }

        Trajectory traj3 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(startingPose.getX(), startingPose.getY()))
                .build();
        mDrive.followTrajectory(traj3);

        mDrive.turnTo(0);

        /*
         * We assume that a tile is 22.75 inches wide without teeth that the teeth are 0.75 inches
         * wide. We assume that the robot is 18 inches long, so the center of the robot is 9 inches
         * in from the back, so therefore the robot itself needs to be at 7.875 inches out to be
         * centered on the first tile. (actually i changed it to 4.5 inches because that works)
         */
        Trajectory traj4 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(4.5, 0, finalHeading))
                .build();
        mDrive.followTrajectory(traj4);
    }

    private void routineGrabOnFieldPixel(double pickupAngle) {
        armSubsystem.applyPosition(onFieldGrabPosition);
        if (isBlueSide) {
            armSubsystem.openRightClaw();
        } else {
            armSubsystem.openLeftClaw();
        }

        //mDrive.turn(pickupAngle);
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(24.25, 17.125 * (isBlueSide ? -1 : 1), pickupAngle))
                .build();
        mDrive.followTrajectory(traj1);

        armSubsystem.applyPosition(onFieldGrabPosition);
        sleep(500);
        if (isBlueSide) {
            armSubsystem.closeRightClaw();
        } else {
            armSubsystem.closeLeftClaw();
        }
        sleep(500);
        armSubsystem.applyPosition(onFieldGrabPosition);

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(4.5, 0, faceBackdropAngle))
                .addTemporalMarker(0.3, () -> armSubsystem.applyPosition(onFieldHighPosition))
                .addTemporalMarker(1.5, () -> armSubsystem.applyPosition(compactPosition))
                .build();
        mDrive.followTrajectory(traj2);
    }

    private void routineWaitAtStart(boolean isLongDistance, boolean isDoingAlternateRoute, boolean isScoringDouble) {
        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(23.5 + 4.5, 0))
                .build();
        mDrive.followTrajectory(traj1);

        double resumeTime = 30.0 - runtime.seconds() - Constants.AUTO_MIN_TIME_NEAR;
        if (isLongDistance) resumeTime -= Constants.AUTO_TIME_ADD_FAR;
        if (isDoingAlternateRoute) resumeTime -= Constants.AUTO_TIME_ADD_FAR_ALT;
        if (isScoringDouble) resumeTime -= Constants.AUTO_TIME_ADD_SCORE_2X;

        sleep((int) (resumeTime * 1000));

        Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineTo(new Vector2d(4.5, 0))
                .build();
        mDrive.followTrajectory(traj2);

        mDrive.turnTo(faceBackdropAngle);
    }

    private void routineApproachBackdrop(boolean isLongDistance, boolean isDoingAlternateRoute) {
        // "Premature optimization is the root of all evil"
        double offsetY1 = (24 * (isLongDistance && !isDoingAlternateRoute ? 3 : 1)) * (isBlueSide ? 1 : -1);
        double offsetY2 = (24 * 3) * (isBlueSide ? 1 : -1);

        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(5, offsetY1, faceBackdropAngle))
                .build();
        mDrive.followTrajectory(traj1);

        /*Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(4.5 + 23.5 * (isLongDistance && isDoingAlternateRoute ? 2 : 1),
                        offsetY1,
                        faceBackdropAngle))
                .build();
        mDrive.followTrajectory(traj2);*/

        if (isLongDistance && isDoingAlternateRoute) {
            double offsetX1 = 2.0 + (24 * 2);

            armSubsystem.applyWristPosition(1.1);

            // Go around the spike marker tile
            mDrive.turnTo(0);

            Trajectory traj3 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(offsetX1, offsetY1, 0))
                    .build();
            mDrive.followTrajectory(traj3);

            mDrive.turnTo(faceBackdropAngle);

            Trajectory traj4 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(offsetX1, offsetY2, faceBackdropAngle))
                    .build();
            mDrive.followTrajectory(traj4);

            mDrive.setPoseEstimate(new Pose2d(64, offsetY2 + 0 * (isBlueSide ? -1 : 1), mDrive.getPoseEstimate().getHeading()));
        }

        // Make sure it's still pointing the right direction
        /*Trajectory traj5 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(
                        new Vector2d(23.5 + 15, 24 * (isLongDistance ? 3 : 1) * (isBlueSide ? 1 : -1)),
                        faceBackdropAngle))
                .build();
        mDrive.followTrajectory(traj5);*/
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

    private void routinePlaceOnBackdrop(boolean isUsingLeftClaw) {
        armSubsystem.applyPosition(backdropPlacePosition);

        double sidewaysPlaceOffset = 0;
        switch (spikeMarkerPosition) {
            case LEFT:
                // Scoring at the left side of the backdrop
                sidewaysPlaceOffset = isUsingLeftClaw ? placeInnerOffset : placeOuterOffset;
                break;

            case CENTER:
                // Scoring at the center of the backdrop
                sidewaysPlaceOffset = isUsingLeftClaw ? placeCenterOffset : -placeCenterOffset;
                break;

            case RIGHT:
                // Scoring at the right side of the backdrop
                sidewaysPlaceOffset = isUsingLeftClaw ? -placeOuterOffset : -placeInnerOffset;
                break;
        }

        /*TrajectoryBuilder traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate());
        if (sidewaysPlaceOffset > 0) {
            traj.strafeRight(sidewaysPlaceOffset);
        } else {
            traj.strafeLeft(Math.abs(sidewaysPlaceOffset));
        }
        mDrive.followTrajectory(traj.build());*/
        //mDrive.strafeRight(sidewaysPlaceOffset);

        placeBackdropPixel(isUsingLeftClaw, !isUsingLeftClaw, sidewaysPlaceOffset, true);
        armSubsystem.applyPosition(compactPosition);
    }

    private void routinePlaceDoubleOnBackdrop(boolean isPrioritizingLeftClaw) {
        /*
         * The pixel in the prioritized claw must be placed at the side corresponding to the spike
         * marker position. The other pixel can go anywhere, as long as it ends up on the backdrop.
         */
        armSubsystem.applyPosition(backdropPlacePosition);
        if (!isPrioritizingLeftClaw) {
            // Place the pixel in the right claw at the corresponding spike marker position
            switch (spikeMarkerPosition) {
                case LEFT:
                    // Scoring at the left side of the backdrop
                    // This has to go twice because there's no way to line up both claws at once
                    placeBackdropPixel(false, true, placeOuterOffset, true);
                    placeBackdropPixel(true, false, -2, false);
                    break;

                case CENTER:
                    // Scoring at the center of the backdrop
                    placeBackdropPixel(true, true, -placeCenterOffset, true);
                    break;

                case RIGHT:
                    // Scoring at the right side of the backdrop
                    placeBackdropPixel(true, true, -placeInnerOffset, true);
                    break;
            }
        } else {
            // Place the pixel in the left claw at the corresponding spike marker position
            switch (spikeMarkerPosition) {
                case LEFT:
                    // Scoring at the left side of the backdrop
                    placeBackdropPixel(true, true, placeInnerOffset, true);
                    break;

                case CENTER:
                    // Scoring at the center of the backdrop
                    placeBackdropPixel(true, true, placeCenterOffset, true);
                    break;

                case RIGHT:
                    // Scoring at the right side of the backdrop
                    // This has to go twice because there's no way to line up both claws at once
                    placeBackdropPixel(true, false, -placeOuterOffset, true);
                    placeBackdropPixel(false, true, 2, false);
                    break;
            }
        }
        armSubsystem.applyPosition(compactPosition);
    }

    private void routinePark(SidePosition parkSide) {
        // Face away from the driver station so that the heading initializes correctly in teleop
        //mDrive.turnTo(0);
        // Move arm rotation motor to zero position so it zeroes correctly in teleop
        armSubsystem.getRotationMotor().setTargetPosition(0);

        // Park on the correct side and face away from driver station
        if (parkSide == SidePosition.INSIDE
                || (parkSide == SidePosition.RIGHT && isBlueSide)
                || (parkSide == SidePosition.LEFT && !isBlueSide)) {
            // Parking at the inside of the backstage
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(34.25 + 24,
                            (24 * (isLongDistance ? 3 : 1) + 10) * (isBlueSide ? 1 : -1),
                            0))
                    .build();
            mDrive.followTrajectory(traj);

        } else if (parkSide == SidePosition.OUTSIDE
                || (parkSide == SidePosition.RIGHT && !isBlueSide) // Some of these conditions are redundant, but intentionally left in for clarity.
                || (parkSide == SidePosition.LEFT && isBlueSide)) {
            // Parking at the outside of the backstage
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(5,
                            (24 * (isLongDistance ? 3 : 1) + 10) * (isBlueSide ? 1 : -1),
                            0))
                    .build();
            mDrive.followTrajectory(traj);

        } else if (parkSide == SidePosition.CENTER) {
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(34.25,
                            (24 * (isLongDistance ? 3 : 1) + 10) * (isBlueSide ? 1 : -1),
                            0))
                    .build();
            mDrive.followTrajectory(traj);
        }

        /*if (parkSide == SidePosition.LEFT
                || (parkSide == SidePosition.OUTSIDE && isBlueSide)
                || (parkSide == SidePosition.INSIDE && !isBlueSide)) {
            // Parking at the left side of the backstage
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .forward((23.5 + sidewaysPlaceOffset) * (isBlueSide ? -1.2 : 1))
                    .build();
            mDrive.followTrajectory(traj);

        } else if (parkSide == SidePosition.RIGHT
                || (parkSide == SidePosition.OUTSIDE && !isBlueSide)
                || (parkSide == SidePosition.INSIDE && isBlueSide)) {
            // Parking at the right side of the backstage
            Trajectory traj = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .forward((23.5 - sidewaysPlaceOffset) * (isBlueSide ? 1 : -1.2))
                    .build();
            mDrive.followTrajectory(traj);
        }*/
    }

    private void routineMoveInPark() {
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

    // Method to place a pixel on a spike line
    private void placeSpikePixel1() {
        armSubsystem.applyPosition(spikePlacePosition);
        sleep(300);
    }
    private void placeSpikePixel2(boolean isUsingRightClaw) {
        if (!isUsingRightClaw) {
            armSubsystem.openLeftClaw();
        } else {
            armSubsystem.openRightClaw();
        }
        sleep(200);
        if (!isUsingRightClaw) {
            armSubsystem.closeLeftClaw();
        } else {
            armSubsystem.closeRightClaw();
        }
        armSubsystem.applyPosition(compactPosition);
    }

    // Method to approach and place a pixel on the backdrop
    private void placeBackdropPixel(boolean isUsingLeftClaw, boolean isUsingRightClaw, double sidewaysOffset, boolean fullApproach) {
        placeBackdropPixel(isUsingLeftClaw, isUsingRightClaw, sidewaysOffset, fullApproach, false);
        //placeBackdropPixel(isUsingLeftClaw, isUsingRightClaw, 0, fullApproach, true);
    }

    private void placeBackdropPixel(boolean isUsingLeftClaw, boolean isUsingRightClaw, double sidewaysOffset, boolean fullApproach, boolean debugStop) {
        double offsetY = 15.5;

        Pose2d approachPose = new Pose2d(34.25 + sidewaysOffset * (isBlueSide ? -1 : 1),
                (24 * (isLongDistance ? 3 : 1) + (fullApproach ? offsetY / 2.0 : offsetY)) * (isBlueSide ? 1 : -1),
                faceBackdropAngle);

        Trajectory traj1 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .lineToLinearHeading(approachPose)
                .build();
        mDrive.followTrajectory(traj1);

        armSubsystem.alignWristToBackdrop();
        armSubsystem.alignWristToAngle(3.0 / 4.0);

        if (fullApproach) {
            mDrive.turnTo(faceBackdropAngle);

            mDrive.setPoseEstimate(approachPose);

            Trajectory traj2 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                    .forward(offsetY / 2.0)
                    .build();
            mDrive.followTrajectory(traj2);
        }

        if (debugStop) requestOpModeStop();

        if (isUsingRightClaw) {
            armSubsystem.openPartlyRightClaw();
        }
        if (isUsingLeftClaw) {
            armSubsystem.openPartlyLeftClaw();
        }
        sleep(400);

        Trajectory traj3 = mDrive.trajectoryBuilder(mDrive.getPoseEstimate())
                .back(6)
                .build();
        mDrive.followTrajectory(traj3);

        if (isUsingRightClaw) {
            armSubsystem.closeRightClaw();
        }
        if (isUsingLeftClaw) {
            armSubsystem.closeLeftClaw();
        }
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
}
