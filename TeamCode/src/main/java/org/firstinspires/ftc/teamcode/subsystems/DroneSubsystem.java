package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class DroneSubsystem extends SubsystemBase {
    private final Servo droneReleaseServo;

    // Constructor method
    public DroneSubsystem(HardwareMap hardwareMap) {
        this.droneReleaseServo = hardwareMap.get(Servo.class, "drone_release_servo");

        this.droneReleaseServo.setDirection(Servo.Direction.FORWARD);
    }

    // Methods
    public void releaseDrone() {
        droneReleaseServo.setPosition(Constants.DRONE_RELEASE_OPEN);
    }

    public void closeDroneRelease() {
        droneReleaseServo.setPosition(Constants.DRONE_RELEASE_CLOSED);
    }
}
