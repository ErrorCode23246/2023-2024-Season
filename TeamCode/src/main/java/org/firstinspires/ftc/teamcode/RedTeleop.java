package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="RedTeleop")
public class RedTeleop extends LinearOpMode {

    // Declare our RobotHardware
    RobotHardware rb = new RobotHardware(this);

    @Override
    public void runOpMode() {

        // Initialize our RobotHardware
        rb.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double ArmPower;
            telemetry.addData("Status", "Running");
            telemetry.update();
            double forward=gamepad1.left_stick_x;
            double strafe=gamepad1.left_stick_y;

            double arm = gamepad2.left_stick_y;
            ArmPower    = Range.clip(arm, -1.0, 1.0) ;
            if (gamepad2.b){//claw
                rb.claw.setPosition(.88);
            } else if (gamepad2.a) {
                rb.claw.setPosition(.12);
            }
            if(gamepad1.b){
                forward=-gamepad1.left_stick_y;
                strafe=gamepad1.left_stick_x;
            }
            rb.turn.setPower(gamepad2.right_stick_y/4);

            boolean turnRight=gamepad1.right_bumper;
            boolean turnLeft=gamepad1.left_bumper;
            double turnAmount =gamepad1.right_stick_x;

            if(gamepad1.b){
                forward=-gamepad1.left_stick_y;
                strafe=gamepad1.left_stick_x;
            }

            if(gamepad1.a){
                strafe/=2;
                forward/=2;
            }


            turnAmount /=2;
            double denominator = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turnAmount), 1);
            rb.FrontRight.setPower((forward-strafe+turnAmount)/denominator);
            rb.FrontLeft.setPower((forward+strafe+turnAmount)/denominator);
            rb.BackLeft.setPower((forward-strafe-turnAmount)/denominator);
            rb.BackRight.setPower((forward+strafe-turnAmount)/denominator);
            rb.ArmMotor.setPower(ArmPower/3);
        }
    }
}