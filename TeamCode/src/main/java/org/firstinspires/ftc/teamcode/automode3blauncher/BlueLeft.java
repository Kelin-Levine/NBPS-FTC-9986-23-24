package org.firstinspires.ftc.teamcode.automode3blauncher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumAutoMode3b;
import org.firstinspires.ftc.teamcode.SidePosition;

@Disabled
@Autonomous(name="Gary Mecanum Autonomous Mode 3b (Blue-Left)", group="Robot")
public class BlueLeft extends MecanumAutoMode3b {
    @Override
    public void runOpMode() {
        isBlueSide = true;
        isLongDistance = false;
        //isDoingAlternateRoute = Constants.AUTO_USE_ALTERNATE_ROUTES;
        //parkSide = Constants.AUTO_PARK_SIDE;
        isWaitingForEnd = false;
        isMovingInPark = true;
        selectedRoutine = Routine.NORMAL;
        super.runOpMode();
    }
}
