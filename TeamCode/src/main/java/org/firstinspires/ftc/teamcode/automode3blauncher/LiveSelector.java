package org.firstinspires.ftc.teamcode.automode3blauncher;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumAutoMode3b;
import org.firstinspires.ftc.teamcode.SidePosition;

import java.util.function.BooleanSupplier;

@Autonomous(name="Gary Mecanum Autonomous Mode 3b (Live Selector)", group="Robot")
public class LiveSelector extends MecanumAutoMode3b {
    /* This class "extends" MecanumAutoMode3b, making it a "subclass" of that class.
     Subclasses automatically inherit all of the variables and methods in the class that they extend.
     You can think of it as if this class contains all of the code in MecanumAutoMode3b, plus the
     code written below.
     That's why variables like 'isBlueSide' can be used without being declared in this class;
     'isBlueSide' is in MecanumAutoMode3b, which means it's in here too.

     MecanumAutoMode3b is an OpMode in itself, and this class exists to add to it with some
     functionality that's not necessarily related to the OpMode, but tells it how to run. By
     extending the original class, this class can add to it directly without cluttering the
     original class.

     When a class extends another class, it is the "subclass". In this case, LiveSelector is the subclass.
     The class that was extended is the "superclass". In this case, MecanumAutoMode3b is the superclass.

     @Override means that a method is meant to replace another method with the same name that is in
     the superclass. For example, the runOpMode() below will run instead of the runOpMode() in
     MecanumAutoMode3b when runOpMode() is called. */

    @Override
    public void runOpMode() {
        // This code contains lambdas. There is an explanation of lambdas in MecanumTeleOpMode3.

        // Defaults in case the routine starts early:
        isBlueSide = false;
        isLongDistance = false;
        isDoingAlternateRoute = Constants.AUTO_USE_ALTERNATE_ROUTES;
        parkSide = Constants.AUTO_PARK_SIDE;
        isWaitingForEnd = false;
        isMovingInPark = false;
        selectedRoutine = Routine.NORMAL;
        //

        telemetry.addData("Auto setup", "(part 1/7)");
        telemetry.addData("Is the robot on the blue alliance?", "");
        telemetry.addData("A/cross", "Yes");
        telemetry.addData("B/circle", "No");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b));
        if (gamepad1.a) {
            isBlueSide = true;
            sleepWhileTrue(() -> gamepad1.a); // Wait until the button is released before moving on
        } else if (gamepad1.b) {
            isBlueSide = false;
            sleepWhileTrue(() -> gamepad1.b); // Wait until the button is released before moving on
        } else {
            /* Because gamepad1 is volatile, its values may change at any moment, even between lines
             of code. Because of this, we can't assume that A or B are still pressed, even though
             they just were. */
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        telemetry.addData("Auto setup", "(part 2/7)");
        telemetry.addData("Is the robot at the far side of the field?", "");
        telemetry.addData("A/cross", "Yes");
        telemetry.addData("B/circle", "No");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b));
        if (gamepad1.a) {
            isLongDistance = true;
            sleepWhileTrue(() -> gamepad1.a);
        } else if (gamepad1.b) {
            isLongDistance = false;
            sleepWhileTrue(() -> gamepad1.b);
        } else {
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        telemetry.addData("Auto setup", "(part 3/7)");
        telemetry.addData("Should the robot do the alternate route?", "");
        telemetry.addData("A/cross", "Yes");
        telemetry.addData("B/circle", "No");
        telemetry.addData("X/square", "Default");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b || gamepad1.x));
        if (gamepad1.a) {
            isDoingAlternateRoute = true;
            sleepWhileTrue(() -> gamepad1.a);
        } else if (gamepad1.b) {
            isDoingAlternateRoute = false;
            sleepWhileTrue(() -> gamepad1.b);
        } else if (gamepad1.x) {
            isDoingAlternateRoute = Constants.AUTO_USE_ALTERNATE_ROUTES;
            sleepWhileTrue(() -> gamepad1.x);
        } else {
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        telemetry.addData("Auto setup", "(part 4/7)");
        telemetry.addData("Where should the robot park?", "");
        telemetry.addData("left dpad", "Left");
        telemetry.addData("right dpad", "Right");
        telemetry.addData("up dpad", "Center");
        telemetry.addData("A/cross", "Outer side");
        telemetry.addData("B/circle", "Inner side");
        telemetry.addData("X/square", "Default");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up));
        if (gamepad1.a) {
            parkSide = SidePosition.OUTSIDE;
            sleepWhileTrue(() -> gamepad1.a);
        } else if (gamepad1.b) {
            parkSide = SidePosition.INSIDE;
            sleepWhileTrue(() -> gamepad1.b);
        } else if (gamepad1.x) {
            parkSide = Constants.AUTO_PARK_SIDE;
            sleepWhileTrue(() -> gamepad1.x);
        } else if (gamepad1.dpad_left) {
            parkSide = SidePosition.LEFT;
            sleepWhileTrue(() -> gamepad1.dpad_left);
        } else if (gamepad1.dpad_right) {
            parkSide = SidePosition.RIGHT;
            sleepWhileTrue(() -> gamepad1.dpad_right);
        } else if (gamepad1.dpad_up) {
            parkSide = SidePosition.CENTER;
            sleepWhileTrue(() -> gamepad1.dpad_up);
        } else {
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        telemetry.addData("Auto setup", "(part 5/7)");
        telemetry.addData("Should the robot wait until the end to score?", "");
        telemetry.addData("A/cross", "Yes");
        telemetry.addData("B/circle", "No");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b));
        if (gamepad1.a) {
            isWaitingForEnd = true;
            sleepWhileTrue(() -> gamepad1.a);
        } else if (gamepad1.b) {
            isWaitingForEnd = false;
            sleepWhileTrue(() -> gamepad1.b);
        } else {
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        telemetry.addData("Auto setup", "(part 6/7)");
        telemetry.addData("Should the robot move out of the way before parking?", "");
        telemetry.addData("A/cross", "Yes");
        telemetry.addData("B/circle", "No");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b));
        if (gamepad1.a) {
            isMovingInPark = true;
            sleepWhileTrue(() -> gamepad1.a);
        } else if (gamepad1.b) {
            isMovingInPark = false;
            sleepWhileTrue(() -> gamepad1.b);
        } else {
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        telemetry.addData("Auto setup", "(part 7/7)");
        telemetry.addData("Which routine should be run?", "");
        telemetry.addData("A/cross", "Normal (50 pts. + 1 backdrop pixel)");
        telemetry.addData("B/circle", "Ambitious (55 pts. + 2 backdrop pixels) (farside only)");
        telemetry.update();
        sleepWhileTrue(() -> !(gamepad1.a || gamepad1.b));
        if (gamepad1.a) {
            selectedRoutine = Routine.NORMAL;
        } else if (gamepad1.b) {
            selectedRoutine = Routine.AMBITIOUS;
        } else {
            telemetry.addData("I didn't get that. Please restart the OpMode and try again.", "");
            telemetry.update();
            sleepWhileTrue(() -> !isStopRequested());
        }

        if (isStopRequested()) return;

        telemetry.setAutoClear(false);
        if (isStarted()) telemetry.addData("STARTING EARLY WITH SOME/ALL DEFAULTS! GOOD LUCK!", "");
        telemetry.addData("Auto setup complete!", "");
        telemetry.addData("On the blue alliance", isBlueSide);
        telemetry.addData("At the farside position", isLongDistance);
        telemetry.addData("Doing the alternate route", isDoingAlternateRoute);
        telemetry.addData("Parking position", parkSide);
        telemetry.addData("Scoring at the end", isWaitingForEnd);
        telemetry.addData("Moving out the way before parking", isMovingInPark);
        telemetry.addData("Selected routine", selectedRoutine);
        telemetry.addData("Please wait for initialization.", "");
        telemetry.addData("", "");
        telemetry.update();

        /* The keyword 'super' contains all methods in the superclass and explicitly refers to them
         as they are written in the superclass, bypassing any overrides in this class.
         super.runOpMode() runs the runOpMode() in MecanumAutoMode3b, not the one in this class.
         That way, this code can continue on with the main routine after this setup. */
        super.runOpMode();
    }

    /* BooleanSupplier is a class that contains a "Runnable": a method stored in a variable.
     BooleanSupplier specifically contains a Runnable that returns a boolean value.
     By using a BooleanSupplier, sleepWhileTrue() is not asking for a boolean variable, but instead
     a method that returns one. The difference is that a normal variable contains only the value,
     while a method contains the condition that determines a value. This way, sleepWhileTrue() can
     run the method to check for changes every time.
     In short, sleepWhileTrue() doesn't take true or false, it takes a condition and runs it to find
     out if it's still true or false every time it needs to check. */
    private void sleepWhileTrue(BooleanSupplier condition) {
        while (condition.getAsBoolean() && !isStopRequested() && !isStarted()) {
            sleep(50);
        }
    }
}
