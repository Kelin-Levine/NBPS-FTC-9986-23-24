package org.firstinspires.ftc.teamcode.automode3blauncher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3b;

@Autonomous(name="Gary Mecanum Autonomous Mode 3b (Red-Left)", group="Robot")
public class RedLeft extends MecanumAutoMode3b {
    @Override
    public void runOpMode() {
        isBlueSide = false;
        isLongDistance = true;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}
