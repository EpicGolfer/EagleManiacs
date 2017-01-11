package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.TouchSensor;


public class Autonomous extends LinearOpMode
{
    DcMotor driveRight;
    DcMotor driveLeft;

    DcMotor frontRight;
    DcMotor frontLeft;
    public void runOpMode() throws InterruptedException {

    }

    DcMotor motorleft;
    DcMotor motorright;


    //@Overridepublic void runOpMode() throws InterruptedException
    {
        motorleft = hardwareMap.dcMotor.get("frontLeft");
        motorright = hardwareMap.dcMotor.get("frontRight");


        motorleft.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();


        DriveForward(0.5);
        sleep(1000);
        StopDriving();


        TurnRight(0.5);
        sleep(3000);
        StopDriving();
    }


    public void DriveForward(double power)
    {
        motorleft.setPower(power);
        motorright.setPower(power);
    }


    public void StopDriving()
    {
        DriveForward(0);
    }


    public void TurnRight(double power)
    {
        motorright.setPower(-power);
        motorleft.setPower(power);
    }

}


