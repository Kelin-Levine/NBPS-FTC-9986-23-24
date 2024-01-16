package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This is a class for a set of motors that make up the arm. It records all motors
 * and servos and can apply a position to all at once, converting it from the
 * arbitrary scale to accurate positions. For more info, see the ArmPosition class.
 */
public class ArmAssembly {

    // Private instance variables
    private final DcMotor liftMotor;
    private final DcMotor travelMotor;
    private final Servo wristServo;

    private double liftTargetPositionScaled;
    private double travelTargetPositionScaled;
    private double wristTargetPositionScaled;

    // Constructor method
    public ArmAssembly(DcMotor liftMotor, DcMotor travelMotor, Servo wristServo) {
        this.liftMotor = liftMotor;
        this.travelMotor = travelMotor;
        this.wristServo = wristServo;

        this.liftTargetPositionScaled = Calculations.encoderToScaleArmLift(liftMotor.getCurrentPosition());
        this.travelTargetPositionScaled = Calculations.encoderToScaleArmTravel(travelMotor.getCurrentPosition());
        this.wristTargetPositionScaled = Calculations.encoderToScaleArmWrist(wristServo.getPosition());
    }

    // Methods
    public void applyPosition(ArmPosition position) {
        applyLiftPosition(position.getLiftAngle());
        applyTravelPosition(position.getTravelPosition());
        applyWristPosition(position.getWristAngle());
    }

    public void applyLiftPosition(double scaled) {
        // The angle for the lift to point at, on a scale of 0 (straight down) to 1 (straight up)
        // Avoid going below the motor's zero position
        // Remember that the lift motor's zero position will likely be above 0 on this scale
        liftTargetPositionScaled = scaled;
        liftMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmLift(scaled)));
    }

    public void applyTravelPosition(double scaled) {
        // The position for the stendo to travel to, on a scale of inches
        // Avoid going below the motor's zero position
        travelTargetPositionScaled = scaled;
        travelMotor.setTargetPosition(Math.max(0, Calculations.scaleToEncoderArmTravel(scaled)));
    }

    public void applyWristPosition(double scaled) {
        // The angle for the wrist to point at, on a scale where 1 is up
        wristTargetPositionScaled = scaled;
        wristServo.setPosition(Calculations.scaleToEncoderArmWrist(scaled));
    }

    // Getter methods
    public DcMotor getLiftMotor() {
        return liftMotor;
    }

    public DcMotor getTravelMotor() {
        return travelMotor;
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public double getLiftTargetPositionScaled() {
        return liftTargetPositionScaled;
    }

    public double getTravelTargetPositionScaled() {
        return travelTargetPositionScaled;
    }

    public double getWristTargetPositionScaled() {
        return wristTargetPositionScaled;
    }
}
