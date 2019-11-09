package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name ="Red Navigate", group = "Red Auton")
public class RedAutonDriveToLine extends LinearOpMode {
    //Declare variables/ objects
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorsensor;
    double color;
    boolean stop;


    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        colorsensor = hardwareMap.colorSensor.get("colorsensor");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        runtime.reset();
       colorsensor.enableLed(true);
       stop = false;

       while(opModeIsActive())
       {


         if(!(colorsensor.red()>32)&& stop == false )
           {
              driveForward(-0.5);
              telemetry.addData("Red", colorsensor.red());
              telemetry.update();
          }
          else {
              driveForward(0.0);
              stop = true;
          }
           telemetry.addData("Red", colorsensor.red());
           telemetry.update();

       }

    }
        public void driveForward ( double power)
        {
            motorLeft.setPower(power);
            motorRight.setPower(power);
        }
    }