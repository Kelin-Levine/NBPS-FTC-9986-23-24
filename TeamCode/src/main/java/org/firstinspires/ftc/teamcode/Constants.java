package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

/*
 * This is a class that contains numbers to be used elsewhere in the code.
 * Keeping the constant numbers separate from the rest of the code makes it easier to configure
 * properties of the robot and keeps things from getting too messy.
 *
 * This class contains static variables. Static variables are a part of the class, not objects made from it.
 * Static variables are used directly from the class's name.
 *
 * This class contains final variables. Final variables cannot be changed by = after being created.
 * If a final variable is a primitive type (such as int or boolean), it cannot be changed at all after being created.
 * Final variables are useful for numbers that need to be used in multiple places but shouldn't change.
 */
public class Constants {

    // Control configuration
    public static final double    ARM_CONTROL_STICK_THRESHOLD = 0.8; // How far the stick controlling the arm must be from its center for the arm to move
    public static final int       ARM_COLOR_DEAD_ZONE        = 200; // The number of encoder ticks that the arm can be away from it's target for the controller to no longer light up
    public static final int       ARM_COLOR_SCALE            = 1000; // The number of encoder ticks over which the arm will light up stronger as it gets farther from its target

    // Component configuration
    public static final boolean   USE_RANGE_SENSOR              = false; // Whether the autonomous portion should try to make use of a range sensor

    public static final double    LEFT_CLAW_CLOSED           = 0.70; // The position of the left claw when it's fully closed
    public static final double    LEFT_CLAW_OPEN             = 0.27; // The position of the left claw when it's fully open
    public static final double    RIGHT_CLAW_CLOSED          = 0.72; // The position of the right claw when it's fully closed
    public static final double    RIGHT_CLAW_OPEN            = 0.29; // The position of the right claw when it's fully open

    public static final double    WRIST_POSITION_UP          = 0.62; // The servo position for the wrist when the arm is down the wrist is pointing straight up

    public static final int       TRAVEL_TICKS_1_INCH        = 32; // The number of encoder ticks for the arm stendo to travel 1 inch

    public static final int       LIFT_TICKS_180_DEGREES     = 2200 * 2; // The number of encoder ticks for the arm lift to travel 180 degrees
    public static final int       LIFT_TICKS_NORTH           = 3600; // The number of arm lift encoder ticks at the north (straight up) position
    public static final int       LIFT_POSITION_MAX          = 3000; // The safe maximum number of arm lift encoder ticks for the arm to swing between
    public static final int       LIFT_POSITION_MAX_OVERRIDE = 5600; // The absolute maximum number of encoder ticks for the arm lift to swing between

    public static final double    DRIVE_TICKS_REVOLUTION     = 537.6;  // The number of encoder ticks in one revolution of a drive motor
    public static final int       DRIVE_TICKS_ROBOT_STRAFE_1 = 48;   // The (positive) number of encoder ticks of each drive motor to strafe the robot 1 inch
    public static final int       DRIVE_TICKS_ROBOT_TURN_90  = 850;  // The (positive) number of encoder ticks of each drive motor to turn the robot 90 degrees
    public static final double    WHEEL_CIRCUMFERENCE_INCHES = (96 / 25.4) * Math.PI; // The approximate circumference of a drive motor's wheel
                                                                              // Wheels are 96mm in diameter, this must be converted to inches
    public static final double    DRIVE_P_COEFFICIENT        = 5.5; // The P coefficient to use for the drivetrain

    // Lens intrinsics
    // UNITS ARE PIXELS
    // Note: These numbers are made up! They should be be configured for the camera on the robot.
    public static final double    LENS_FX       = 578.272;
    public static final double    LENS_FY       = 578.272;
    public static final double    LENS_CX       = 320;
    public static final double    LENS_CY       = 240;

    // UNITS ARE METERS
    public static final double    APRILTAG_SIZE = 0.166;

    // TeleOp configuration
    public static final double    DRIVE_POWER_MULTIPLIER     = 0.9; // The multiplier scale on the robot's drivetrain power
    public static final double    ARM_LIFT_POWER             = 0.6; // The amount of power that the arm lifts with
    public static final double    ARM_TRAVEL_POWER           = 0.4; // The amount of power that the arm extends/retracts with

    // Autonomous configuration
    public static final boolean   AUTO_USE_ALTERNATE_ROUTES     = false; // Whether the autonomous routines should use their alternate routes
    public static final Boolean   AUTO_FLIP_CLAWS               = null;  // Whether to flip which sides of the claw to use in auto (set to null for automatic)
    public static final SidePosition    AUTO_PARK_SIDE          = SidePosition.CENTER; // What side of the backstage to park in during auto

    public static final double    AUTO_DRIVE_POWER_MULTIPLIER   = 0.45; // The multiplier scale on the robot's drivetrain power (in autonomous)
    public static final double    AUTO_ARM_LIFT_POWER           = 0.6; // The amount of power that the arm lifts with (in autonomous)
    public static final double    AUTO_ARM_TRAVEL_POWER         = 0.4; // The amount of power that the arm extends/retracts with (in autonomous)
    public static final SidePosition FALLBACK_SPIKE_MARKER_POSITION = SidePosition.CENTER; // The spike marker position to fall back to if one cannot be found

    // Autonomous vision configuration
    //public static final Point VISION_REGION1_TOPLEFT_ANCHOR_POINT = new Point(445,350); // The top-left corner of the area of the right spike marker in the image at the start of auto
    //public static final Point VISION_REGION2_TOPLEFT_ANCHOR_POINT = new Point(140,320); // The top-left corner of the area of the middle spike marker in the image at the start of auto
    //public static final Point VISION_REGION3_TOPLEFT_ANCHOR_POINT = new Point(253,98); // The top-left corner of the area of the left spike marker in the image at the start of auto
    public static final Point VISION_REGION1_CENTER_ANCHOR_POINT = new Point(480,365); // The center of the area of the right spike marker in the image at the start of auto
    public static final Point VISION_REGION2_CENTER_ANCHOR_POINT = new Point(150,335); // The center of the area of the middle spike marker in the image at the start of auto
    public static final int   VISION_SUBREGION_WIDTH    = 20;  // The width of the area to look for a spike marker in the vision image
    public static final int   VISION_SUBREGION_HEIGHT   = 30;  // The height of the area to look for a spike marker in the vision image
    public static final int   VISION_REGION1_SUBREGION_COUNT     = 3;   // The number of subregions to look for the right spike marker
    public static final int   VISION_REGION2_SUBREGION_COUNT     = 5;   // The number of subregions to look for the middle spike marker
    public static final int   VISION_SUBREGION_DISTANCE = 10;  // The distance between each subregion
    public static final int   VISION_CHROMA_THRESHOLD   = 160; // The minimum chroma value for one of two chroma values to be likely to have a spike marker
    public static final int   VISION_APRILTAG_CHECKS    = 3;   // The number of times that all AprilTags must be seen in a row for the robot to proceed from the long distance
    public static final double VISION_APRILTAG_TIMEOUT  = 8;   // How much time can be remaining in the auto routine before waiting for AprilTags to not be blocked at long distance times out

    // Autonomous vision display configuration
    public static final Scalar    BLUE  = new Scalar(0, 0, 255); // The color of boxes drawn blue on the vision display
    public static final Scalar    GREEN = new Scalar(0, 255, 0); // The color of boxes drawn green on the vision display
    public static final int       VISION_BOX_THICKNESS = 4;      // The thickness of boxes drawn on the vision display

}
