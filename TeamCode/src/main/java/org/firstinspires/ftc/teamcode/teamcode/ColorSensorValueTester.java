package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "ColorSensorValueTester", group = "Tester")
public class ColorSensorValueTester extends LinearOpMode {

   // ColorSensor colorSensorFront;
    ColorSensor colorsensor;

    @Override
    public void runOpMode()
    {
        colorsensor = hardwareMap.colorSensor.get("colorsensor");
        //colorSensorFront = hardwareMap.colorSensor.get("colorSensorFront");

        waitForStart();
        while(opModeIsActive())
        {
           // telemetry.addData("Front ColorSensor Blue Value: " + colorSensorFront.blue(), ", Red Value: " + colorSensorFront.red());
            telemetry.addData("Back  ColorSensor Value value: " + colorsensor.blue(),", Red Value: " + colorsensor.red());
            telemetry.update();
        }
    }
}
