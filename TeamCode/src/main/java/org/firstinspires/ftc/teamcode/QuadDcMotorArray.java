package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * This is a data class for a set of driving motors. It records four motors
 * and can apply a set of power values to all four motors at once.
 */
public class QuadDcMotorArray {

    // Private instance variables (private variables that are in an instance of this class)
    private final DcMotor leftFrontMotor; // 'final' means that this variable will never be set again after it is set in the constructor.
    private final DcMotor rightFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    private double powerMultiplier;


    // Constructor methods
    public QuadDcMotorArray(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.powerMultiplier = 1.0;
    }

    public QuadDcMotorArray(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor, double powerMultiplier) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.powerMultiplier = powerMultiplier;
    }

    // Methods
    public void setPower(QuadMotorValues<Double> power) {
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
        return (leftFrontMotor.isBusy() || leftBackMotor.isBusy() || rightFrontMotor.isBusy() || rightBackMotor.isBusy());
    }
    // Hack version that turns up false faster
    public boolean isMotorsBusyFast() {
        return ((leftFrontMotor.isBusy() && leftBackMotor.isBusy()) || (rightFrontMotor.isBusy() && rightBackMotor.isBusy()));
    }

    // Method to zero all four motors as RUN_USING_ENCODER motors
    public void zeroMotorsRunUsingEncoder() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to zero all four motors as RUN_TO_POSITION motors
    public void zeroMotorsRunToPosition() {
        zeroRunToPositionMotor(leftFrontMotor, powerMultiplier);
        zeroRunToPositionMotor(rightFrontMotor, powerMultiplier);
        zeroRunToPositionMotor(leftBackMotor, powerMultiplier);
        zeroRunToPositionMotor(rightBackMotor, powerMultiplier);
    }

    // Method to zero a RUN_TO_POSITION motor
    private void zeroRunToPositionMotor(DcMotor motor, double power) {
        // reset, then power, then position, then mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(power);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
