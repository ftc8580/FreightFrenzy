package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name="CDAutonBaseTest", group="Linear Opmode")
//@Disabled
public class CDAutonBaseTest extends CDAutonBase {
    @Override
    public void executeAuton() {
        myTurret.setTurretDirection("center");
        sleep(200);
        myDuckSpinner.setDuckSpinnerPower(-.6);
        sleep(2500);
        myDuckSpinner.setDuckSpinnerPower(0);
        myTurret.setTurretDirection("center");
        myElevator.setElevatorPosition(getDuckDeliveryLocation(duckLocation, myElevator));
        myIntake.setIntakePower(.4);
        sleep(1000);
        myIntake.setIntakePower(0);
    }

}
