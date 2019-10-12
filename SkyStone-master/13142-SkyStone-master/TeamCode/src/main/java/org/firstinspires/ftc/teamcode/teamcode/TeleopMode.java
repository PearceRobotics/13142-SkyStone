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
    private DcMotor fourBar2;
    private Servo leftServo;
    private Servo rightServo;
    private Servo intakeLeft;
    private Servo intakeRight;

    //Variables
    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;
    static final double IntakeUp = 0.0;
    static final double IntakeDown = 1.0;
    static final int PositionA = 0;
    static final int PositionB = 100;
    static final int PositionC = 200;
    static final int PositionD = 300;
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
        fourBar = hardwareMap.dcMotor.get("lifter");
        fourBar2 = hardwareMap.dcMotor.get("lifter2");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        fourBar2.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.REVERSE);

        positionA = true;
        positionB = false;
        positionC = false;
        positionD = false;

        //fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // fourBar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //  fourBar2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            if(gamepad2.left_bumper)
            {
                intakeMechanism(IntakeUp);
            }
            //Moves intake down
            if(gamepad2.right_bumper)
            {
                intakeMechanism(IntakeDown);
            }

           /* if(gamepad2.a)
            {
                positionA();
            }
            if(gamepad2.b)
            {
                positionB();
            }
            if(gamepad2.x)
            {
                positionC();
            }
            if(gamepad2.y)
            {
                positionD();
            }*/
            //raises and lowers four bar linkage
            if(gamepad2.left_trigger> .01)
            {
                raiseFourBar(-gamepad2.left_trigger*3/4);
               telemetry.addData("current position",fourBar.getCurrentPosition());
               telemetry.update();
            }
            if(gamepad2.right_trigger> 0.1) {
                raiseFourBar(gamepad2.right_trigger*3/4);
                telemetry.addData("current position",fourBar.getCurrentPosition());
                telemetry.update();
            }
            else
            {
                brakeFourBar();
            }

        }
    }
    public void brakeFourBar()
    {
        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fourBar2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
  /*  public void positionA()
    {
        if(positionB = true)
        {
            positionCode(-PositionB);
            positionB = false;
        }
        if(positionC = true);
        {
            positionCode(-PositionC);
            positionC = false;
        }
        if(positionD = true)
        {
            positionCode(-PositionD);
            positionD = false;
        }
        positionA = true;
    }
    public void positionB()
    {
        if(positionA = true)
        {
            positionCode((PositionB - PositionA));
            positionA = false;
        }
        if(positionC = true)
        {
            positionCode((PositionB - PositionC));
            positionC = false;
        }
        if(positionD = true)
        {
            positionCode((PositionB - PositionD));
            positionD = false;
        }
        positionB = true;
    }
    public void positionC()
    {
        if(positionA = true)
        {
            positionCode((PositionC - PositionA));
            positionA = false;
        }
        if(positionB = true)
        {
            positionCode((PositionC -PositionB));
            positionB = false;
        }
        if(positionD = true)
        {
            positionCode((PositionC-PositionD));
            positionD = false;
        }
        positionC = true;
    }
    public void positionD()
    {
        if(positionA = true)
        {
            positionCode((PositionD - PositionA));
            positionA = false;
        }
        if(positionB = true)
        {
            positionCode((PositionD - PositionB));
            positionB = false;
        }
        if(positionC = true)
        {
            positionCode((PositionD - PositionC));
            positionC = false;
        }
        positionD = true;
    }

    public void positionCode( int TargetPosition)
    {
        fourBar.setTargetPosition(TargetPosition);
        fourBar2.setTargetPosition(TargetPosition);
        while(fourBar.isBusy()|| fourBar2.isBusy() && opModeIsActive()) {
            //Loop body can be empty
        }
        fourBar.setPower(0);
    }*/


}
