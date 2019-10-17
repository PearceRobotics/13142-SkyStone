package org.firstinspires.ftc.teamcode.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Teleop", group = "Teleop" )
public class TeleopMode extends LinearOpMode
{

    //Declare motors and servos
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotorEx fourBar;
    private DcMotorEx fourBar2;
    private Servo leftServo;
    private Servo rightServo;
    private Servo intakeLeft;
    private Servo intakeRight;


    //Variables
    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;
    static final double IntakeUp = 0.0;
    static final double IntakeDown = 1.0;
    static final double armSpeed = 1;
    static boolean positionA;
    static boolean positionB;
    static boolean positionC;
    static boolean positionD;


    @Override
    public void runOpMode() throws InterruptedException
    {
        Context myApp = hardwareMap.appContext;
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        fourBar = (DcMotorEx)hardwareMap.get(DcMotor.class, "lifter");
        fourBar2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "lifter2");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        fourBar2.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.REVERSE);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fourBar2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        positionA = true;
        positionB = false;
        positionC = false;
        positionD = false;

        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ////1120 ticks per rotation

        waitForStart();
        PIDCoefficients pidOrig = fourBar.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrig2 = fourBar2.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        while(opModeIsActive())
        {
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.addData("P,I,D (orig2)", "%.04f, %.04f, %.0f",
                    pidOrig2.p, pidOrig2.i, pidOrig2.d);
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
            if(gamepad2.left_bumper)
            {
                intakeMechanism(IntakeUp);
            }
            //Moves intake down
            if(gamepad2.right_bumper)
            {
                intakeMechanism(IntakeDown);
            }

            if(gamepad2.a)
            {
                moveArm(-fourBar.getCurrentPosition() -5, armSpeed);
            }
            if(gamepad2.b)
            {
                moveArm(25, armSpeed);
            }
            if(gamepad2.x)
            {
                moveArm(50, armSpeed);
            }
            if(gamepad2.y)
            {
                moveArm(100, armSpeed);
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
    public void raiseFourBar(double liftSpeed)
    {
        fourBar.setPower(liftSpeed);
        fourBar2.setPower(liftSpeed);
    }


    public void armEncoderReset(){
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveArm(int position, double power)
    {
        armEncoderReset();

        fourBar.setTargetPosition(position);
        fourBar2.setTargetPosition(position);

        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBar2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fourBar.setPower(power);
        fourBar2.setPower(power);

        while(fourBar.isBusy() || fourBar2.isBusy()){
            telemetry.addData("Moving to", position);
            telemetry.addData("From", fourBar.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Arm Status", "done," + position);
        telemetry.update();
        fourBar.setPower(0);
        fourBar2.setPower(0);
    }


}
