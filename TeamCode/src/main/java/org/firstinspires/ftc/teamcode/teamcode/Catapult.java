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
        telemetry.addData("initialized", 1);
        telemetry.update();
        Catapult.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("kind of working", 1);
            telemetry.update();
            if(gamepad1.a)
            {
              Catapult.setPosition(1.0);
                telemetry.addData("up", 1);
                telemetry.update();
            }
            if(gamepad1.b)
            {
                Catapult.setPosition(0);
                telemetry.addData("down", 1);
                telemetry.update();
            }

        }
    }

}
