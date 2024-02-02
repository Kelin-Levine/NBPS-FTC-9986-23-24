package org.firstinspires.ftc.teamcode.automode3launcher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3;

@Autonomous(name="Gary Mecanum Autonomous Mode 3 (Blue-Right)", group="Robot")
public class BlueRight extends MecanumAutoMode3 {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = true;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}
