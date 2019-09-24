package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name ="Drive Foward", group = "Beginner Auton")
public class AutonDriveFoward extends LinearOpMode {
    //Declare variables/ objects
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < 3.0) {
            driveForward(1.0);
        }
        while (opModeIsActive() && runtime.seconds() > 3.0) {
            driveForward(0.0);
        }
    }
        public void driveForward ( double power)
        {
            motorLeft.setPower(power);
            motorRight.setPower(power);
        }
    }