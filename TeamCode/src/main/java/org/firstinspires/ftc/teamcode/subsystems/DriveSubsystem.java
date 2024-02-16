package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Calculations;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.QuadMotorValues;

/*
 * This is a data class for a set of driving motors. It records four motors
 * and can apply a set of power values to all four motors at once.
 */
public class DriveSubsystem extends SubsystemBase {

    // Private instance variables (private variables that are in an instance of this class)
    private final Motor leftFrontMotor; // 'final' means that this variable will never be set again after it is set in the constructor.
    private final Motor rightFrontMotor;
    private final Motor leftBackMotor;
    private final Motor rightBackMotor;
    private final IMU imu;

    private double powerMultiplier;
    private boolean isFieldCentric = true;


    // Constructor methods
    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, 1.0);
    }

    public DriveSubsystem(HardwareMap hardwareMap, double powerMultiplier) {
        this.powerMultiplier = powerMultiplier;

        this.leftFrontMotor = new Motor(hardwareMap, "left_front_drive");
        this.rightFrontMotor = new Motor(hardwareMap, "right_front_drive");
        this.leftBackMotor = new Motor(hardwareMap, "left_back_drive");
        this.rightBackMotor = new Motor(hardwareMap, "right_back_drive");

        this.leftFrontMotor.setInverted(false);
        this.leftBackMotor.setInverted(false);
        this.rightFrontMotor.setInverted(true);
        this.rightBackMotor.setInverted(true);

        this.leftFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.leftFrontMotor.setRunMode(Motor.RunMode.RawPower);
        this.leftBackMotor.setRunMode(Motor.RunMode.RawPower);
        this.rightFrontMotor.setRunMode(Motor.RunMode.RawPower);
        this.rightBackMotor.setRunMode(Motor.RunMode.RawPower);

        this.leftFrontMotor.resetEncoder();
        this.leftBackMotor.resetEncoder();
        this.rightFrontMotor.resetEncoder();
        this.rightBackMotor.resetEncoder();

        // Retrieve the IMU from the hardware map
        /* Usually the device name would be "imu" for the Control Hub's IMU, but we use "imu1" to
        retrieve the IMU available in some older Expansion Hubs because our Control Hub is borked.
        This may cause problems on robots with no or a new Expansion Hub. */
        this.imu = hardwareMap.get(IMU.class, "imu1");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu.initialize(parameters);
    }

    // Methods
    public void drive(GamepadEx gamepad) {
        double axial = -gamepad.getLeftY();
        double lateral = gamepad.getLeftX();
        double yaw = gamepad.getRightX();
        drive(axial, lateral, yaw);
    }
    public void drive(double axial, double lateral, double yaw) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        QuadMotorValues<Double> drivePower;
        if (isFieldCentric) {
            drivePower = Calculations.mecanumDriveisFieldCentric(axial, lateral, yaw, heading);
        } else {
            drivePower = Calculations.mecanumDriveRobotCentric(axial, lateral, yaw);
        }
        setPower(drivePower);
    }

    public void setFullSpeed() {
        powerMultiplier = Constants.DRIVE_POWER_MULTIPLIER;
    }

    public void setSlowSpeed() {
        powerMultiplier = Constants.DRIVE_POWER_MULTIPLIER_SLOW;
    }

    public void toggleIsFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

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

    // Method to zero all four motors
    public void zeroMotors() {
        leftFrontMotor.resetEncoder();
        rightFrontMotor.resetEncoder();
        leftBackMotor.resetEncoder();
        rightBackMotor.resetEncoder();
    }

    // Method to zero IMU heading
    public void zeroHeading() {
        imu.resetYaw();
    }

    // Getter methods
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getPowerMultiplier() {
        return powerMultiplier;
    }

    public boolean getIsFieldCentric() {
        return isFieldCentric;
    }

    // Setter methods
    public void setPowerMultiplier(double multiplier) {
        powerMultiplier = multiplier;
    }

    public void setIsFieldCentric(boolean isFieldCentric) {
        this.isFieldCentric = isFieldCentric;
    }
}
