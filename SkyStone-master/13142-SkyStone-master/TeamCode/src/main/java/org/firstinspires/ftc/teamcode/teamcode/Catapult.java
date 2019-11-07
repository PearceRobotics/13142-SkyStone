package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "catapult", group = "A")
public class Catapult extends LinearOpMode {

    public Servo Catapult;

    public double up = 1.0;
    public double down = 0;

    @Override
    public void runOpMode(){
        Catapult = hardwareMap.get(Servo.class, "Servo");

        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
              Catapult.setPosition(up);
            }
            if(gamepad1.b)
            {
                Catapult.setPosition(down);
            }

        }
    }

}
