package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
class DistanceTest extends LinearOpMode {
    DistanceSensor distance;
    DcMotor motor;

    @Override
    public void runOpMode() {
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        motor = hardwareMap.get(DcMotor.class, "Motor");

        waitForStart();
        while (opModeIsActive()) {
            if (distance.getDistance(DistanceUnit.CM) < 10) {
                motor.setPower(0.3);
            } else {
                motor.setPower(0);
            }
        }
    }
}
