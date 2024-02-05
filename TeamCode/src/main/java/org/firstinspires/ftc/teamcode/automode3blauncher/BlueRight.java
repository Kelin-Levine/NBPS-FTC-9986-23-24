package org.firstinspires.ftc.teamcode.automode3blauncher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3b;

@Autonomous(name="Gary Mecanum Autonomous Mode 3b (Blue-Right)", group="Robot")
public class BlueRight extends MecanumAutoMode3b {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = true;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}
