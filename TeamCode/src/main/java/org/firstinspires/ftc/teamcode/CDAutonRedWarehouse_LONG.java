package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="CDAutonRedWarehouse_LONG", group="Linear Opmode")
//@Disabled
public class CDAutonRedWarehouse_LONG extends CDAutonBase {
    @Override
    public void initTokenWeDoNotSee() {
        duckWeDoNotSee = 3;
    }

    public double intakepos;
    static final double AUTON_LONG_SPEED = 0.6;
    static final double AUTON_LONG_TURN = 0.5;

    @Override
    public void executeAuton() {
        ElapsedTime myTimer = new ElapsedTime();
        myTimer.reset();

        // Start back at right edge
        myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -3, 5);
        myChassis.encoderDriveStraight(AUTON_LONG_SPEED,3,10.0);
        myChassis.encoderDriveTurn(AUTON_LONG_TURN, -35, 10);
        myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -5, 18);

        sleep(100);
        myTurret.setTurretDirection("left", true);
        myElevator.setElevatorPosition(getDuckDeliveryLocation(duckLocation, myElevator));

        // go to elevator
        myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -13, 18);
        myChassis.encoderDriveStrafe(0.4, -2, 18);
        //myChassis.encoderDriveTurn(AUTON_LONG_TURN, -55, 10);
        myIntake.setIntakePower(.5);
        sleep(1000);
        myIntake.setIntakePower(0);
        //myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, 55, 10);

        // Return
        myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 20, 18);
        myChassis.encoderDriveTurn(AUTON_LONG_TURN, 35, 10);
        myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 12, 18);

        // Enter warehouse
        myTurret.setTurretDirection("center", true);
        sleep(100);
        myTurret.setTurretDirection("center", false);
        myElevator.setElevatorPosition(myElevator.elevatorposground);
        myChassis.encoderDriveStraight(AUTON_LONG_SPEED,20,10.0);
        myChassis.encoderDriveStraight(AUTON_LONG_SPEED,6,10.0);
        //position above is just inside warehouse

        //turn on intake move forward then back
        //while (intakepos > 6) {
            myIntake.setIntakePower(-1.0);
            myChassis.encoderDriveStraight(0.4, 12, 10.0);
            myIntake.setIntakePower(0);
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED, -12, 10.0);
        //}

        //check block there (less than 6 then block is there)
        intakepos = myDistanceSensor.getIntakeDistance();
        telemetry.addData("IntakeDist", "%.2f", intakepos);
        telemetry.update();

        if ((myTimer.seconds()<25) & (intakepos < 6)) {
            myElevator.setElevatorPosition(myElevator.elevatorpostop);
            myTurret.setTurretDirection("left", false);
            //go to hub
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 2, 5);
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED,-26,10.0);
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -12, 10);
            myChassis.encoderDriveTurn(AUTON_LONG_TURN, -35, 10);
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -16, 18);
            //myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, -55, 10);
            myIntake.setIntakePower (.4);
            sleep(1000);
            myIntake.setIntakePower(0);
            //myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, 55, 10);
            // Return
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 16, 18);
            myChassis.encoderDriveTurn(AUTON_LONG_TURN, 35, 10);
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 12, 18);
            // Enter warehouse
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED,26,10.0);
        }

        if ((myTimer.seconds()<25) & (intakepos > 6)) {
            myTurret.setTurretDirection("center", false);
            myElevator.setElevatorPosition(myElevator.elevatorposground);
            myIntake.setIntakePower(-.4);
            myChassis.encoderDriveStraight(0.4, 18, 10.0);
            myIntake.setIntakePower(0);
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED, -18, 10.0);
        }

        if ((myTimer.seconds()<25) & (intakepos < 6)) {
            myElevator.setElevatorPosition(myElevator.elevatorpostop);
            myTurret.setTurretDirection("left", false);
            //go to hub
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 2, 5);
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED,-26,10.0);
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -12, 10);
            myChassis.encoderDriveTurn(AUTON_LONG_TURN, -35, 10);
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, -16, 18);
            //myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, -55, 10);
            myIntake.setIntakePower (.4);
            sleep(1000);
            myIntake.setIntakePower(0);
            //myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, 55, 10);
            // Return
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 16, 18);
            myChassis.encoderDriveTurn(AUTON_LONG_TURN, 35, 10);
            myChassis.encoderDriveStrafe(AUTON_LONG_SPEED, 12, 18);
            // Enter warehouse
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED,26,10.0);
        }

        //park
        myElevator.setElevatorPosition(myElevator.elevatorposmiddle);
        myChassis.encoderDriveStrafe(AUTON_LONG_SPEED,-25,10.0);
        myTurret.setTurretDirection("right", false);
        myChassis.encoderDriveStraight(AUTON_LONG_SPEED,22,10.0);
        myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, 90, 10);
        myTurret.setTurretDirection("center", false);
        if (intakepos > 6) {
            myElevator.setElevatorPosition(myElevator.elevatorposground);
            myIntake.setIntakePower(-1.0);
            myChassis.encoderDriveStraight(0.4, 12, 10.0);
            myIntake.setIntakePower(0);
            myChassis.encoderDriveStraight(AUTON_LONG_SPEED, -12, 10.0);
        }



//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED,-5,10.0);
//        myElevator.setElevatorPosition(7);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED,45,10.0);
//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED,28,10.0);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED,-24,10.0);
//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED,24,10.0);
//        myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, 90, 10);
    }
}


//Sample Code
//Drive
//myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED, 30, 10.0); //Move forward 30 inches with 5 second timeout. Negative is backward.
//myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, 10, 5.0); //Move right 10 inches with 5 second timeout. Negative is left.
//myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, -180, 5.0); //Turn left 180 degrees with a 5 second timeout. Positive is right.

//Elevator
//myElevator.setElevatorPosition(getDuckDeliveryLocation(duckLocation, myElevator)); // This will set to position based on cap location. Copy as is.
//myElevator.setElevatorPosition(26); //Move elevator to middle position. 7 is ground. 14 is bottom. 26 is middle. 39 is top. Do not set outside of range 2.5-39.

//Turret
//myTurret.setTurretDirection("center"); //Set turret position to center, right or left

//Intake and Duck Spinner
//myIntake.setIntakePower(1.0);
//myDuckSpinner.setDuckSpinnerPower(.7);
//sleep(250); //optional pause after each move in milliseconds
//Remember to set power back to zero


//OLD FROM 1ST COMPETITION
//    public void executeAuton() {
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, -3, 5);
//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED, -24, 10.0);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, -12, 15);
//        myTurret.setTurretDirection("left", true);
//        myElevator.setElevatorPosition(getDuckDeliveryLocation(duckLocation, myElevator));
//        myIntake.setIntakePower(.4);
//        sleep(1000);
//        myIntake.setIntakePower(0);
//        myTurret.setTurretDirection("center", true);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, 15, 15);
//        myElevator.setElevatorPosition(14);
//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED,40,10.0);
//        myElevator.setElevatorPosition(7);
//        myIntake.setIntakePower(-.6);
//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED,8,10.0);
//        myIntake.setIntakePower(0);
//        myChassis.encoderDriveStraight(CDDriveChassisAuton.DRIVE_SPEED,-48,10.0);
//        myElevator.setElevatorPosition(39);
//        myTurret.setTurretDirection("left", true);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, -15, 15);
//        myIntake.setIntakePower(.4);
//        sleep(1000);
//        myIntake.setIntakePower(0);
//        myTurret.setTurretDirection("center", true);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, 15, 15);
//        myElevator.setElevatorPosition(14);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED, -36, 15);
//        myChassis.encoderDriveTurn(CDDriveChassisAuton.TURN_SPEED, -90, 10);
//        myChassis.encoderDriveStrafe(CDDriveChassisAuton.DRIVE_SPEED,-25,10.0);
//        myElevator.setElevatorPosition(7);
//    }
//}