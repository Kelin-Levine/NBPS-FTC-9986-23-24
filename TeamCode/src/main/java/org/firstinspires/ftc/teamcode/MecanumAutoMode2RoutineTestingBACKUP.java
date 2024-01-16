package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class MecanumAutoMode2RoutineTestingBACKUP {

    // Variables

    // Pre-programmed positions
    private final ArmPosition compactPosition = new ArmPosition(0.18, 0, 1);

    // Working variables
    private Boolean isCameraOpened;

    // OpMode objects
    private final LinearOpMode mainOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Hardware objects
    ModernRoboticsI2cRangeSensor frontRangeSensor;
    private OpenCvCamera frontCamera;
    private Servo leftClawServo;
    private Servo rightClawServo;
    private QuadDcMotorArray motors;
    private ArmAssembly armAssembly;


    // Constructor
    public MecanumAutoMode2RoutineTestingBACKUP(LinearOpMode mainOpMode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.mainOpMode = mainOpMode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        isCameraOpened = null;
    }

    // Method to initialize the robot
    public void initialize() {
        // Initialize the motors/hardware. The names used here must correspond to the names set on the driver control station.
        if (Constants.USE_RANGE_SENSOR) {
            frontRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range_sensor");
        }

        WebcamName frontWebcam = hardwareMap.get(WebcamName.class, "front_webcam");
        int frontCameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        DcMotor armLiftMotor = hardwareMap.get(DcMotor.class, "arm_lift_motor");
        DcMotor armTravelMotor = hardwareMap.get(DcMotor.class, "arm_travel_motor");
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
                // TODO: research and verify camera image width and height
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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = new QuadDcMotorArray(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, Constants.AUTO_DRIVE_POWER_MULTIPLIER);

        motors.zeroMotorsRunToPosition();

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
        zeroRunToPositionMotor(armTravelMotor, Constants.AUTO_ARM_TRAVEL_POWER);

        armLiftMotor.setDirection((DcMotor.Direction.FORWARD));
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroRunToPositionMotor(armLiftMotor, Constants.AUTO_ARM_LIFT_POWER);

        armAssembly = new ArmAssembly(armLiftMotor, armTravelMotor, armWristServo);

        armAssembly.applyPosition(compactPosition);

        // Initialization complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Method to run the autonomous routine
    public void runRoutine(boolean isBlueSide, Gamepad gamepad1) {
        final ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("Starting autonomous routine.", "");
        telemetry.update();

        // START
        SidePosition spikeMarkerPosition = SidePosition.UNDEFINED;

        // FIND SPIKE MARKER
        // Wait until camera has opened
        while (isCameraOpened == null) sleep(50);

        // Find the spike marker
        if (isCameraOpened) {
            // If the camera opened...
            // Start the spike vision pipeline
            telemetry.addData("Looking for spike marker...", "");
            telemetry.update();
            VisionPipelineAutoMode2Spike visionPipeline = new VisionPipelineAutoMode2Spike(isBlueSide, true);
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

        } else {
            // If the camera failed to open...
            // Fall back to the fallback spike marker position
            spikeMarkerPosition = Constants.FALLBACK_SPIKE_MARKER_POSITION;
            telemetry.addData("Camera failed to open.", "Falling back to fallback spike position (" + spikeMarkerPosition + ")");
            telemetry.update();
        }

        // WAIT FOR INPUT TO PROGRESS
        telemetry.addData("", "");
        telemetry.addData("Vision part 1 (spike markers) complete.", "Press south button on gamepad1 to continue with vision part 2 (apriltag counting).");
        telemetry.update();
        while (!gamepad1.a && mainOpMode.opModeIsActive()) {
            sleep(50);
        }
        if (mainOpMode.isStopRequested()) return;

        // COUNT APRILTAGS
        if (isCameraOpened) {
            // If the camera opened...
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

            // Wait until apriltags have been spotted
            for (int i = 0; i < 100; i++) {
                sleep(100);
                telemetry.addData("# of apriltags spotted:", aprilTagPipeline.getLatestDetections().size());
                telemetry.update();
            }

        } else {
            // If the camera failed to open...
            // Proceed without looking
            telemetry.addData("Camera failed to open. I hope there's no robot in the way!", "");
            telemetry.update();
        }

        // COMPLETE
        telemetry.addData("Autonomous routine complete.", "( " + Math.ceil(runtime.seconds()) + " seconds)");
        telemetry.addData("Waiting for OpMode termination.", "");
        telemetry.update();
    }

    // Method to zero a RUN_TO_POSITION motor
    private void zeroRunToPositionMotor(DcMotor motor, double power) {
        // reset, then power, then position, then mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(power);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Sleep method
    // This is the same shorthand for Thread.sleep(long) that LinearOpMode (the OpMode class) uses.
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
