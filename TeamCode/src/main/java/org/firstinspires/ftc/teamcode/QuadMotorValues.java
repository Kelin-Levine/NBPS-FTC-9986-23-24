package org.firstinspires.ftc.teamcode;

/*
 * This is a data class for a set of motor power values. It records an amount of power for four driving motors.
 */
public class QuadMotorValues<T> {

    // Private instance variables
    private T leftFrontValue;
    private T rightFrontValue;
    private T leftBackValue;
    private T rightBackValue;


    // Constructor method
    public QuadMotorValues(T leftFrontValue, T rightFrontValue, T leftBackValue, T rightBackValue) {
        this.leftFrontValue = leftFrontValue;
        this.rightFrontValue = rightFrontValue;
        this.leftBackValue = leftBackValue;
        this.rightBackValue = rightBackValue;
    }

    // Getter methods
    public T getLeftFrontValue() {
        return leftFrontValue;
    }

    public T getRightFrontValue() {
        return rightFrontValue;
    }

    public T getLeftBackValue() {
        return leftBackValue;
    }

    public T getRightBackValue() {
        return rightBackValue;
    }

    // Setter methods
    public void setLeftFrontValue(T leftFrontValue) {
        this.leftFrontValue = leftFrontValue;
    }

    public void setRightFrontValue(T rightFrontValue) {
        this.rightFrontValue = rightFrontValue;
    }

    public void setLeftBackValue(T leftBackValue) {
        this.leftBackValue = leftBackValue;
    }

    public void setRightBackValue(T rightBackValue) {
        this.rightBackValue = rightBackValue;
    }
}
