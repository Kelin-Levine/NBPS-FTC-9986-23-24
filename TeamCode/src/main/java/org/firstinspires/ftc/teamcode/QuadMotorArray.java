package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This is a data class for a set of driving motors. It records four motors
 * and can apply a set of power values to all four motors at once.
 */
public class QuadMotorArray {

    // Private instance variables (private variables that are in an instance of this class)
    private final DcMotor leftFrontMotor; // 'final' means that this variable will never be set again after it is set in the constructor.
    private final DcMotor rightFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    private double powerMultiplier;


    // Constructor methods
    public QuadMotorArray(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.powerMultiplier = 1.0;
    }

    public QuadMotorArray(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor, double powerMultiplier) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.powerMultiplier = powerMultiplier;
    }

    // Methods
    public void setPower(QuadMotorValues power) {
        leftFrontMotor.setPower(power.getLeftFrontValue() * powerMultiplier);
        rightFrontMotor.setPower(power.getRightFrontValue() * powerMultiplier);
        leftBackMotor.setPower(power.getLeftBackValue() * powerMultiplier);
        rightBackMotor.setPower(power.getRightBackValue() * powerMultiplier);
    }

    public void setPower(double power) {
        leftFrontMotor.setPower(power * powerMultiplier);
        rightFrontMotor.setPower(power * powerMultiplier);
        leftBackMotor.setPower(power * powerMultiplier);
        rightBackMotor.setPower(power * powerMultiplier);
    }

    // Getter methods
    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    // Setter methods
    public void setPowerMultiplier(double multiplier) {
        powerMultiplier = multiplier;
    }
}
