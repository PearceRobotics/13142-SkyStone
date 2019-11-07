package org.firstinspires.ftc.teamcode.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Teleop", group = "Teleop" )
public class TeleopMode extends LinearOpMode {

    //Declare motors and servos
   public static DcMotor motorLeft;
   public static DcMotor motorRight;
   public static DcMotorEx fourBar;
   public static DcMotorEx fourBar2;
   public static Servo leftServo;
   public static Servo rightServo;
   public static  Servo intakeLeft;
   public static Servo intakeRight;
   public static DigitalChannel limitSwitch;





    //Variables
    static final double maxPosition = 0.0;
    static final double minPosition = 1.0;
    static final double IntakeUp = 0.0;
    static final double IntakeDown = 1.0;
    static final double armSpeed = 1;
    static final int ToleranceAd = 20;
    static boolean positionA;
    static boolean positionB;
    static boolean positionC;
    static boolean positionD;

    static boolean tankDrive = true;
    static boolean arcadeDrive = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Context myApp = hardwareMap.appContext;
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        fourBar = (DcMotorEx) hardwareMap.get(DcMotor.class, "lifter");
        fourBar2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "lifter2");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        limitSwitch =hardwareMap.digitalChannel.get("limitSwitch");

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

        waitForStart();
        PIDCoefficients pidOrig = fourBar.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidOrig2 = fourBar2.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive()) {

            if(tankDrive) {
                motorLeft.setPower(-gamepad2.left_stick_y);
                motorRight.setPower(-gamepad2.right_stick_y);
            }
            if(arcadeDrive) {
                double leftPower;
                double rightPower;
                double drive = -gamepad2.left_stick_y;
                double turn = gamepad2.right_stick_x;
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);
            }

            //platform moving servos up
            if (gamepad2.dpad_up) {
                platformMoverPosition(maxPosition);
            }
            //platform moving servos down
            else if (gamepad2.dpad_down) {
                platformMoverPosition(minPosition);
            }

            //Moves intake up
            if (gamepad2.left_bumper) {
                intakeMechanism(IntakeUp);
            }
            //Moves intake down
            if (gamepad2.right_bumper) {
                intakeMechanism(IntakeDown);
            }

            //Arm Code
            if (gamepad2.a) {
                moveArm(185 + ToleranceAd, armSpeed, 20);
            }
            if (gamepad2.b) {
                moveArm(70 + ToleranceAd, armSpeed, 20);
            }
            if (gamepad2.x) {
                moveArm(125 + ToleranceAd, armSpeed, 25);
            }
            if (gamepad2.y) {
                moveArm(155 + ToleranceAd, armSpeed, 15);
            }

            if (gamepad2.right_trigger > 0.1) {
                raiseFourBar(gamepad2.right_trigger/2);
            }
            if (gamepad2.left_trigger > 0.1) {
                raiseFourBar(-gamepad2.left_trigger/4);
            }




            if (gamepad2.back)
            {
                useSwitch();
            }

            if(gamepad2.start)
            {
                if(arcadeDrive)
                {
                    arcadeDrive = false;
                    tankDrive = true;
                }
                else if(tankDrive)
                {
                    tankDrive = false;
                    arcadeDrive = true;
                }
            }
        }
    }

    public void platformMoverPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public void intakeMechanism(double position) {
        intakeLeft.setPosition(position);
        intakeRight.setPosition(position);
    }

    public void raiseFourBar(double liftSpeed) {
        fourBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourBar2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fourBar.setPower(liftSpeed);
            fourBar2.setPower(liftSpeed);
    }


    public void armEncoderReset() {
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveArm(int position, double power, int tolerance) {
        if (fourBar.getCurrentPosition() > 200) {
        } else {
            armEncoderReset();

            fourBar.setPower(power);
            fourBar2.setPower(power);

            fourBar.setTargetPositionTolerance(tolerance + ToleranceAd);
            fourBar2.setTargetPositionTolerance(tolerance + ToleranceAd);

            fourBar.setTargetPosition(position);
            fourBar2.setTargetPosition(position);

            fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fourBar2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (fourBar.isBusy() || fourBar2.isBusy()) {
                telemetry.addData("Moving to", position);
                telemetry.addData("From", fourBar.getCurrentPosition());
                telemetry.addData("Power", fourBar.getPower());
                telemetry.update();
            }
            telemetry.addData("Arm Status", "done," + position);
            telemetry.update();
            fourBar.setPower(0);
            fourBar2.setPower(0);
        }


    }
    public void useSwitch()
    {
        fourBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fourBar2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (limitSwitch.getState() == true)
        {
            fourBar.setPower(-.5);
            fourBar2.setPower(-.5);
        }
        fourBar.setPower(0);
        fourBar2.setPower(0);
    }
}
