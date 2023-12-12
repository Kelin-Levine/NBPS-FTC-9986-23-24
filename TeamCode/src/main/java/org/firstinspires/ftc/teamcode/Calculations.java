package org.firstinspires.ftc.teamcode;

/*
 * This is a class that contains methods to perform specific calculations.
 * Keeping the calculations separate from the rest of the code keeps things from getting too messy,
 * and allows complex calculations to be reused easily.
 *
 * This class contains static methods. Static methods are a part of the class, not objects made from it.
 * Static methods are called directly from the class's name.
 */
public class Calculations {

    // Mecanum drive calculations
    public static QuadMotorValues mecanumDrive(double axial, double lateral, double yaw) {
        // Combine the direction requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        return new QuadMotorValues(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // Joystick to arm target angle calculations
    public static int vectorToArmPositionHalf(float vx, float vy) {
        // Have you taken pre-calc yet?

        // Old math:
        //armInputTarget = Math.min(Math.max((int) (Math.atan2(Math.abs(gamepad1.right_stick_x), gamepad1.right_stick_y) / Math.PI * 3400), 0), 3400);

        // Turn the vector into an angle
        double angle = Math.atan2(Math.abs(vx), vy) / Math.PI;
        // Scale the angle to the arm's encoder
        angle *= Constants.liftTicks180Degrees;
        // Get an offset to account for motor's zero position
        angle -= Constants.liftTicks180Degrees - Constants.liftTicksNorth;
        // Cast angle to int and clamp within allowed range
        return Math.min(Math.max((int) angle, 0), Constants.liftPositionMax);
    }

    public static int vectorToArmPositionFull(float vx, float vy) {
        // Turn the vector into an angle
        double angle = Math.atan2(vx, vy) / Math.PI;
        // Make the angle loop around
        if (angle < 0) {
            angle += 2;
        }
        // Scale the angle to the arm's encoder
        angle *= Constants.liftTicks180Degrees;
        // Get an offset to account for motor's zero position
        angle -= Constants.liftTicks180Degrees - Constants.liftTicksNorth;
        // Cast angle to int and clamp within allowed range
        return Math.min(Math.max((int) angle, 0), Constants.liftPositionMaxOverride);
    }

    // Position scaling calculations
    // The angle for the lift to point at, on a scale of 0 (straight down) to 1 (straight up)
    // Remember that the lift motor's zero position will likely be above 0 on this scale
    public static int scaleToEncoderArmLift(double scale) {
        return (int) (scale * Constants.liftTicks180Degrees - (Constants.liftTicks180Degrees - Constants.liftTicksNorth));
    }
    public static double encoderToScaleArmLift(int encoder) {
        return (double) (encoder + (Constants.liftTicks180Degrees - Constants.liftTicksNorth)) / Constants.liftTicks180Degrees;
    }

    // The position for the stendo to travel to, on a scale of inches
    public static int scaleToEncoderArmTravel(double scale) {
        return (int) (scale * Constants.travelTicks1Inch);
    }
    public static double encoderToScaleArmTravel(int encoder) {
        return (double) encoder / Constants.travelTicks1Inch;
    }

    // The angle for the wrist to point at, on a scale where 1 is up
    public static double scaleToEncoderArmWrist(double scale) {
        return scale - (1 - Constants.wristPositionTop);
    }
    public static double encoderToScaleArmWrist(double encoder) {
        return encoder + (1 - Constants.wristPositionTop);
    }

}
