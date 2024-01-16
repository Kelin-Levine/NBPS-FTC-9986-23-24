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
    public static QuadMotorValues<Double> mecanumDriveRobotCentric(double axial, double lateral, double yaw) {
        // Combine the direction requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level.

        //lateral *= 1.1; // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1);

        double leftFrontPower = (axial + lateral + yaw) / denominator;
        double rightFrontPower = (axial - lateral - yaw) / denominator;
        double leftBackPower = (axial - lateral + yaw) / denominator;
        double rightBackPower = (axial + lateral - yaw) / denominator;

        // Send calculated power to wheels
        return new QuadMotorValues<>(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public static QuadMotorValues<Double> mecanumDriveisFieldCentric(double axial, double lateral, double yaw, double heading) {
        // Rotate the movement direction counter to the robot's rotation
        double rotLateral = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double rotAxial = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        //rotLateral *= 1.1;  // Counteract imperfect strafing

        // Combine the direction requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level.

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotAxial) + Math.abs(rotLateral) + Math.abs(yaw), 1);

        double leftFrontPower = (rotAxial + rotLateral + yaw) / denominator;
        double rightFrontPower = (rotAxial - rotLateral - yaw) / denominator;
        double leftBackPower = (rotAxial - rotLateral + yaw) / denominator;
        double rightBackPower = (rotAxial + rotLateral - yaw) / denominator;

        // Send calculated power to wheels
        return new QuadMotorValues<>(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // Joystick to arm target angle calculations
    public static int vectorToArmPositionHalf(float vx, float vy) {
        // Have you taken pre-calc yet?

        // Old math:
        //armInputTarget = Math.min(Math.max((int) (Math.atan2(Math.abs(gamepad1.right_stick_x), gamepad1.right_stick_y) / Math.PI * 3400), 0), 3400);

        // Turn the vector into an angle
        double angle = Math.atan2(Math.abs(vx), vy) / Math.PI;
        // Scale the angle to the arm's encoder
        angle *= Constants.LIFT_TICKS_180_DEGREES;
        // Get an offset to account for motor's zero position
        angle -= Constants.LIFT_TICKS_180_DEGREES - Constants.LIFT_TICKS_NORTH;
        // Cast angle to int and clamp within allowed range
        return Math.min(Math.max((int) angle, 0), Constants.LIFT_POSITION_MAX);
    }

    public static int vectorToArmPositionFull(float vx, float vy) {
        // Turn the vector into an angle
        double angle = Math.atan2(vx, vy) / Math.PI;
        // Make the angle loop around
        if (angle < 0) {
            angle += 2;
        }
        // Scale the angle to the arm's encoder
        angle *= Constants.LIFT_TICKS_180_DEGREES;
        // Get an offset to account for motor's zero position
        angle -= Constants.LIFT_TICKS_180_DEGREES - Constants.LIFT_TICKS_NORTH;
        // Cast angle to int and clamp within allowed range
        return Math.min(Math.max((int) angle, 0), Constants.LIFT_POSITION_MAX_OVERRIDE);
    }

    // Position scaling calculations
    // The distance for a drive motor to spin to, on a scale of inches
    public static int inchesToEncoderDrive(double inches) {
        return (int) (inches * (Constants.DRIVE_TICKS_REVOLUTION / Constants.WHEEL_CIRCUMFERENCE_INCHES));
    }
    public static double encoderToInchesDrive(int encoder) {
        return (encoder * Constants.WHEEL_CIRCUMFERENCE_INCHES) / Constants.DRIVE_TICKS_REVOLUTION;
    }

    // Positions for the drivetrain to spin to in order to strafe the robot, in increments of 1 inch
    public static QuadMotorValues<Integer> inchesStrafeToEncodersDrive(double inches) {
        int driveTicks = (int) (Constants.DRIVE_TICKS_ROBOT_STRAFE_1 * inches);
        return new QuadMotorValues<>(driveTicks, -driveTicks, -driveTicks, driveTicks);
    }
    public static double encodersAverageToInchesStrafeDrive(QuadMotorValues<Integer> encoders) {
        double driveTicksAverageAbs = (Math.abs(encoders.getLeftFrontValue()) + Math.abs(encoders.getRightFrontValue()) + Math.abs(encoders.getLeftBackValue()) + Math.abs(encoders.getRightBackValue())) / 4.0;
        double countsAbs = driveTicksAverageAbs / Constants.DRIVE_TICKS_ROBOT_STRAFE_1;
        return countsAbs * (encoders.getLeftFrontValue() < 0 ? -1 : 1);
    }

    // Positions for the drivetrain to spin to in order to turn the robot, in increments of 90 degrees
    public static QuadMotorValues<Integer> clockwise90TurnsToEncodersDrive(double count) {
        int driveTicks = (int) (Constants.DRIVE_TICKS_ROBOT_TURN_90 * count);
        return new QuadMotorValues<>(driveTicks, -driveTicks, driveTicks, -driveTicks);
    }
    public static double encodersAverageToClockwise90TurnsDrive(QuadMotorValues<Integer> encoders) {
        double driveTicksAverageAbs = (Math.abs(encoders.getLeftFrontValue()) + Math.abs(encoders.getRightFrontValue()) + Math.abs(encoders.getLeftBackValue()) + Math.abs(encoders.getRightBackValue())) / 4.0;
        double countsAbs = driveTicksAverageAbs / Constants.DRIVE_TICKS_ROBOT_TURN_90;
        return countsAbs * (encoders.getLeftFrontValue() < 0 ? -1 : 1);
    }

    // The angle for the lift to point at, on a scale of 0 (straight down) to 1 (straight up)
    // Remember that the lift motor's zero position will likely be above 0 on this scale
    public static int scaleToEncoderArmLift(double scale) {
        return (int) (scale * Constants.LIFT_TICKS_180_DEGREES - (Constants.LIFT_TICKS_180_DEGREES - Constants.LIFT_TICKS_NORTH));
    }
    public static double encoderToScaleArmLift(int encoder) {
        return (double) (encoder + (Constants.LIFT_TICKS_180_DEGREES - Constants.LIFT_TICKS_NORTH)) / Constants.LIFT_TICKS_180_DEGREES;
    }

    // The position for the stendo to travel to, on a scale of inches
    public static int scaleToEncoderArmTravel(double scale) {
        return (int) (scale * Constants.TRAVEL_TICKS_1_INCH);
    }
    public static double encoderToScaleArmTravel(int encoder) {
        return (double) encoder / Constants.TRAVEL_TICKS_1_INCH;
    }

    // The angle for the wrist to point at, on a scale where 1 is up
    public static double scaleToEncoderArmWrist(double scale) {
        return scale - (1 - Constants.WRIST_POSITION_UP);
    }
    public static double encoderToScaleArmWrist(double encoder) {
        return encoder + (1 - Constants.WRIST_POSITION_UP);
    }

}
