package org.firstinspires.ftc.teamcode;

/**
 * Created by Nick on 1/12/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="org.firstinspires.ftc.teamcode.TestDriveUno: Linear OpMode", group="Linear Opmode")

public class MainRobot {
    DcMotor.RunMode devModeL;
    DcMotor.RunMode devModeR;
    DcMotor.RunMode devModeF;
    DcMotor.RunMode devModeA;

    DcMotorController Utilitycontroller;
    DcMotorController rightContoller;
    DcMotorController leftcontroller;

    ServoController gatecontroller;

    DcMotor rightfront;
    DcMotor leftfront;
    DcMotor backright;
    DcMotor backleft;

    DcMotor launch;
    DcMotor lift;


    int numOpLoops = 0;

    public void init()
    {
        RobotLog.w("****DEBUG INIT LINE81 BEGINNING****");






    }



}
