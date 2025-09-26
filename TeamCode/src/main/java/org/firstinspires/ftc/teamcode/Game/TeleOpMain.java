package org.firstinspires.ftc.teamcode.Game;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcDrive;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Tele-Op Main", group="Linear OpMode")
@Disabled
public class Main extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcDrive frontLeftDrive = null;
    private DcDrive backLeftDrive = null;
    private DcDrive frontRightDrive = null;
    private DcDrive backRightDrive = null;


    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcDrive.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcDrive.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcDrive.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcDrive.class, "back_right_drive");

        frontLeftDrive.setDirection(DcDrive.Direction.REVERSE);
        backLeftDrive.setDirection(DcDrive.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeftDrive.setPower(y + x + rx);
            backLeftDrive.setPower(y - x + rx);
            frontRightDrive.setPower(y - x - rx);
            backRightDrive.setPower(y + x - rx);

        }
    }

}