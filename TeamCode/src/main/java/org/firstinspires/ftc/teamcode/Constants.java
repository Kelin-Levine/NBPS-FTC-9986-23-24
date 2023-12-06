package org.firstinspires.ftc.teamcode;

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
    public static final double armControlStickThreshold = 0.8; // How far the stick controlling the arm must be from its center for the arm to move
    public static final int armColorDeadZone = 200; // The number of encoder ticks that the arm can be away from it's target for the controller to no longer light up
    public static final int armColorScale = 1000; // The number of encoder ticks over which the arm will light up stronger as it gets farther from its target

    // Component configuration
    public static final double wristPositionUp = 9.5; // The servo position for the wrist to point up
    public static final int travelTicks1Inch = 25436 / 4; // The number of encoder ticks for the arm stendo to travel 1 inch
    public static final int liftTicks180Degrees = 2200 * 2; // The number of encoder ticks for the arm lift to travel 180 degrees
    public static final int liftTicksNorth = 3300; // The number of arm lift encoder ticks at the north (straight up) position
    public static final int liftPositionMax = 3000; // The safe maximum number of arm lift encoder ticks for the arm to swing between
    public static final int liftPositionMaxOverride = 5600; // The absolute maximum number of encoder ticks for the arm lift to swing between
    //public static final int liftPositionMin = 0; // The minimum number of encoder ticks for the arm lift to swing between

    // TeleOp configuration
    public static final double drivePowerMultiplier = 0.6; // The multiplier scale on the robot's drivetrain
    public static final double armLiftPower = 0.6; // The amount of power that the arm lifts with
    public static final double armTravelPower = 0.4; // The amount of power that the arm extends/retracts with

}
