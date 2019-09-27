package org.firstinspires.ftc.teamcode.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop", group = "Teleop" )
public class TeleopMode extends LinearOpMode
{

    //Declare motors and servos
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private Servo leftServo;
    private Servo rightServo;

    //Variables
    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;
    double intakespeed = 0.75;


    @Override
    public void runOpMode() throws InterruptedException
    {
        Context myApp = hardwareMap.appContext;
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive())
        {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            if(gamepad1.left_bumper)
            {
                rightServo.setPosition(maxPosition);
                leftServo.setPosition(maxPosition);

            }
            else if(gamepad1.right_bumper)
            {
                rightServo.setPosition(minPosition);
                leftServo.setPosition(minPosition);
            }

            if(gamepad2.y)
            {
                intakeOutake(intakespeed);
            }
            if(gamepad2.a)
            {
                intakeOutake(-intakespeed);
            }
            else
            {
                intakeOutake(0);
            }

        }
    }
    public void intakeOutake(double power)
    {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

}
