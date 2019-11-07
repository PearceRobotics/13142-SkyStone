package org.firstinspires.ftc.teamcode.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name= "MagneticLimitsSwitch", group = "test" )
public class MagneticLimitsSwitch extends LinearOpMode {


  public static DigitalChannel limitSwitch;

    @Override
    public void runOpMode() {

        limitSwitch =hardwareMap.digitalChannel.get("limitSwitch");

        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Status",limitSwitch.getState());
            telemetry.update();
        }



    }



}
