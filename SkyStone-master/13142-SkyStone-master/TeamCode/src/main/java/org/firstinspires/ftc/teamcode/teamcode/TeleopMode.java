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
    private DcMotor fourBar;
    private Servo leftServo;
    private Servo rightServo;
    private Servo intakeLeft;
    private Servo intakeRight;

    //Variables
    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;
    static final double lifterUp = 0.0;
    static final double lifterDown = 1.0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        Context myApp = hardwareMap.appContext;
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        fourBar = hardwareMap.dcMotor.get("lifter");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            //tank drive
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //platform moving servos up
            if(gamepad1.left_bumper)
            {
                platformMoverPosition(maxPosition);
            }
            //platform moving servos down
            else if(gamepad1.right_bumper)
            {
                platformMoverPosition(minPosition);
            }

            //Moves intake up
            if(gamepad2.y)
            {
                intakeMechanism(lifterUp);
            }
            //Moves intake down
            if(gamepad2.a)
            {
                intakeMechanism(lifterDown);
            }
            //raises and lowers four bar linkage
            if(gamepad2.left_trigger> .01)
            {
                raiseFourBar(-gamepad2.left_trigger);
            }
            if(gamepad2.right_trigger> 0.1) {
                raiseFourBar(gamepad2.right_trigger);
            }
            else
            {
                fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

        }
    }
    public void platformMoverPosition(double position)
    {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
    public void intakeMechanism(double position)
    {
        intakeLeft.setPosition(position);
        intakeRight.setPosition(position);
    }
    public void raiseFourBar(double intakeSpeed)
    {
        fourBar.setPower(intakeSpeed);
    }

}
