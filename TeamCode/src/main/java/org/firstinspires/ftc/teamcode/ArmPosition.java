package org.firstinspires.ftc.teamcode;

/*
 * This is a class for a position for the arm to go to.
 * It records target positions for the three parts of the arm. Positions are recorded as numbers on
 * a scale, not directly as encoder positions.
 *
 * When a position is applied, it is automatically converted from the scale to the corresponding
 * encoder position. This way, positions won't have to be redone whenever a change is made that
 * affects encoder readings; as long as the constants class is configured correctly, the same value
 * on the scale will always correspond to the same physical position on the robot.
 */
public class ArmPosition {

    // Private instance variables
    private final double liftAngle; // The angle for the lift to point at, on a scale of 0 (straight down) to 1 (straight up)
    private final int travelPosition; // The position for the stendo to travel to, on a scale of inches
    private final double wristAngle; // The angle for the wrist to point at, on a scale where 1 is up

    // Constructor method
    public ArmPosition(double liftAngle, int travelPosition, double wristAngle) {
        this.liftAngle = liftAngle;
        this.travelPosition = travelPosition;
        this.wristAngle = wristAngle;
    }

    // Getter methods
    public double getLiftAngle() {
        return liftAngle;
    }

    public int getTravelPosition() {
        return travelPosition;
    }

    public double getWristAngle() {
        return wristAngle;
    }
}
