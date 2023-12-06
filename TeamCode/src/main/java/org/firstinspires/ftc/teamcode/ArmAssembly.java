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

    // Constructor method
    public ArmAssembly(DcMotor liftMotor, DcMotor travelMotor, Servo wristServo) {
        this.liftMotor = liftMotor;
        this.travelMotor = travelMotor;
        this.wristServo = wristServo;
    }

    // Methods
    public void applyPosition(ArmPosition position) {
        // The angle for the lift to point at, on a scale of 0 (straight down) to 1 (straight up)
        // Avoid going below the motor's zero position
        // Remember that the lift motor's zero position will likely be above 0 on this scale
        liftMotor.setTargetPosition(Math.max(0, (int) (position.getLiftAngle() * Constants.liftTicks180Degrees - (Constants.liftTicks180Degrees - Constants.liftTicksNorth))));
        // The position for the stendo to travel to, on a scale of inches
        // Avoid going below the motor's zero position
        travelMotor.setTargetPosition(Math.max(0, position.getTravelPosition() * Constants.travelTicks1Inch));
        // The angle for the wrist to point at, on a scale where 1 is up
        wristServo.setPosition(position.getWristAngle() - (1 - Constants.wristPositionUp));
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
}
