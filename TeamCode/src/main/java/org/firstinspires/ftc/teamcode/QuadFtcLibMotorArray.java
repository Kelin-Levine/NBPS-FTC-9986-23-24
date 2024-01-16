package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;

/*
 * This is a data class for a set of driving motors. It records four motors
 * and can apply a set of power values to all four motors at once.
 */
public class QuadFtcLibMotorArray {

    // Private instance variables (private variables that are in an instance of this class)
    private final Motor leftFrontMotor; // 'final' means that this variable will never be set again after it is set in the constructor.
    private final Motor rightFrontMotor;
    private final Motor leftBackMotor;
    private final Motor rightBackMotor;

    private double powerMultiplier;


    // Constructor methods
    public QuadFtcLibMotorArray(Motor leftFrontMotor, Motor rightFrontMotor, Motor leftBackMotor, Motor rightBackMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.powerMultiplier = 1.0;
    }

    public QuadFtcLibMotorArray(Motor leftFrontMotor, Motor rightFrontMotor, Motor leftBackMotor, Motor rightBackMotor, double powerMultiplier) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.powerMultiplier = powerMultiplier;
    }

    // Methods
    public void setPower(QuadMotorValues<Double> power) {
        leftFrontMotor.set(power.getLeftFrontValue() * powerMultiplier);
        rightFrontMotor.set(power.getRightFrontValue() * powerMultiplier);
        leftBackMotor.set(power.getLeftBackValue() * powerMultiplier);
        rightBackMotor.set(power.getRightBackValue() * powerMultiplier);
    }

    public void setPower(double power) {
        leftFrontMotor.set(power * powerMultiplier);
        rightFrontMotor.set(power * powerMultiplier);
        leftBackMotor.set(power * powerMultiplier);
        rightBackMotor.set(power * powerMultiplier);
    }

    public void setTargetPosition(QuadMotorValues<Integer> position) {
        leftFrontMotor.setTargetPosition(position.getLeftFrontValue());
        rightFrontMotor.setTargetPosition(position.getRightFrontValue());
        leftBackMotor.setTargetPosition(position.getLeftBackValue());
        rightBackMotor.setTargetPosition(position.getRightBackValue());
    }

    public void setTargetPosition(int position) {
        leftFrontMotor.setTargetPosition(position);
        rightFrontMotor.setTargetPosition(position);
        leftBackMotor.setTargetPosition(position);
        rightBackMotor.setTargetPosition(position);
    }

    // Method to get if any of the four motors are busy
    public boolean isMotorsBusy() {
        return (leftFrontMotor.motor.isBusy() || leftBackMotor.motor.isBusy() || rightFrontMotor.motor.isBusy() || rightBackMotor.motor.isBusy());
    }
    // Hack version that turns up false faster
    public boolean isMotorsBusyFast() {
        return ((leftFrontMotor.motor.isBusy() && leftBackMotor.motor.isBusy()) || (rightFrontMotor.motor.isBusy() && rightBackMotor.motor.isBusy()));
    }

    // Method to zero all four motors
    public void zeroMotors() {
        leftFrontMotor.resetEncoder();
        rightFrontMotor.resetEncoder();
        leftBackMotor.resetEncoder();
        rightBackMotor.resetEncoder();
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
