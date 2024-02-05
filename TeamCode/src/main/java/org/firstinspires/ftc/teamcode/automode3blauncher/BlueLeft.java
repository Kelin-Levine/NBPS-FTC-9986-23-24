package org.firstinspires.ftc.teamcode.automode3blauncher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumAutoMode3b;

@Autonomous(name="Gary Mecanum Autonomous Mode 3b (Blue-Left)", group="Robot")
public class BlueLeft extends MecanumAutoMode3b {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = false;
        selectedRoutine = Routine.NORMAL;
        isDoingAlternateRoute = false;
        super.runOpMode();
    }
}
