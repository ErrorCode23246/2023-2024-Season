package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {

    private LinearOpMode myOpMode = null;

    public DcMotor BackLeft = null;
    public DcMotor BackRight = null;
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotorSimple ArmMotor = null;

    public Servo claw = null;
    public CRServo turn = null;

    private static final double MID_SERVO = 0.5;
    private static final double HAND_SPEED = 0.02;
    private static final double ARM_UP_POWER = 0.45;
    private static final double ARM_DOWN_POWER = -0.45;

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        BackLeft = myOpMode.hardwareMap.dcMotor.get("BackLeft");
        BackRight = myOpMode.hardwareMap.dcMotor.get("BackRight");
        FrontLeft = myOpMode.hardwareMap.dcMotor.get("FrontLeft");
        FrontRight = myOpMode.hardwareMap.dcMotor.get("FrontRight");
        ArmMotor = myOpMode.hardwareMap.get(DcMotorSimple.class, "ArmMotor");

        claw = myOpMode.hardwareMap.servo.get("claw");
        turn = myOpMode.hardwareMap.crservo.get("turn");

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // Rest of the class methods...


    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     */
    public void driveRobot(double forward, double strafe, double turnAmount) {
        // Combine drive and turn for blended motion.
        double FrontRight  = forward-strafe+turnAmount;
        double FrontLeft = forward+strafe+turnAmount;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(FrontLeft), Math.abs(FrontRight));
        if (max > 1.0)
        {
            FrontRight /= max;
            FrontLeft /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(FrontLeft, FrontRight, 1, 1);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param frontLeftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param frontRightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double backLeftWheel, double backRightWheel, double frontLeftWheel, double frontRightWheel) {
        // Output the values to the motor drives.
        FrontLeft.setPower(frontLeftWheel);
        BackLeft.setPower(backLeftWheel);
        FrontRight.setPower(frontRightWheel);
        BackRight.setPower(backRightWheel);
    }


    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        ArmMotor.setPower(power);
    }

}
