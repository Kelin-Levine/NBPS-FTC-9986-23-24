package org.firstinspires.ftc.teamcode;

/*
 * This is a data class for a set of motor power values. It records an amount of power for four driving motors.
 */
public class QuadMotorValues {

    // Private instance variables
    private double leftFrontValue;
    private double rightFrontValue;
    private double leftBackValue;
    private double rightBackValue;


    // Constructor method
    public QuadMotorValues(double leftFrontValue, double rightFrontValue, double leftBackValue, double rightBackValue) {
        this.leftFrontValue = leftFrontValue;
        this.rightFrontValue = rightFrontValue;
        this.leftBackValue = leftBackValue;
        this.rightBackValue = rightBackValue;
    }

    // Getter methods
    public double getLeftFrontValue() {
        return leftFrontValue;
    }

    public double getRightFrontValue() {
        return rightFrontValue;
    }

    public double getLeftBackValue() {
        return leftBackValue;
    }

    public double getRightBackValue() {
        return rightBackValue;
    }

    // Setter methods
    public void setLeftFrontValue(double leftFrontValue) {
        this.leftFrontValue = leftFrontValue;
    }

    public void setRightFrontValue(double rightFrontValue) {
        this.rightFrontValue = rightFrontValue;
    }

    public void setLeftBackValue(double leftBackValue) {
        this.leftBackValue = leftBackValue;
    }

    public void setRightBackValue(double rightBackValue) {
        this.rightBackValue = rightBackValue;
    }
}
