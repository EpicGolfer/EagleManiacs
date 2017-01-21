import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="org.firstinspires.ftc.teamcode.TestDriveUno: Linear OpMode", group="Linear Opmode")


public class RobotMain extends OpMode {


    double tilt = 0;
    double trayDelta = 0.0037;

    DcMotor.RunMode devModeL;
   DcMotor.RunMode devModeR;
   DcMotor.RunMode devModeF;
   DcMotor.RunMode devModeA;
    
    //Here, we define our motor controllers
    //This lets the code know we have 3 DC conts. and their names
    //Names must match setup file on phone.  I think.
    DcMotorController rightController;
    DcMotorController leftController;
    DcMotorController utilityController;

    //Similar, but now for servo controller
    ServoController gatecontroller;

    //These define the actual motors. Still just setup
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor frontLeft;

    DcMotor launcher;
    DcMotor lift;
    Servo gate;


    //Leave this line. It just keeps track of how many times this runs
    int numOpLoops = 0;

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        //Use init() to setup stuff.
        
        //so RobotLog.w("TEXT") writes whatever text to the log on the phone
        //We used this a lot for debugging last year, because it's and easy
        //way to see if your code at this point actually ran
        //I'm leaving this one line as an example, and I'll take out
        //all the other debug lines
        RobotLog.w("****DEBUG INIT LINE81 BEGINNING****");

        //These lines let the code know which controllers are which, from a
        //wiring standpoint. Just use 
        //hardwareMap.dcMotorController.get("yourControllerName")
        //however you have it in the phone setup file
        //This means that now rightController, leftController, and frontController
        //are actual DC controllers, ready to be used
        rightController = hardwareMap.dcMotorController.get("rightController");
        leftController = hardwareMap.dcMotorController.get("leftController");
        gatecontroller= hardwareMap.servoController.get("gatecontroller");
        utilityController = hardwareMap.dcMotorController.get("utilityController");

        backRight = hardwareMap.dcMotor.get("driveRight");
        backLeft = hardwareMap.dcMotor.get("driveLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");

        launcher = hardwareMap.dcMotor.get("catapult");
        lift = hardwareMap.dcMotor.get("lift");

        gate = hardwareMap.servo.get("gate");

    }

    @Override
    public void init_loop() {

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);


        //starting position of main tray
        //getPosition() gets the current position of a servo
        //it makes trayTiltL and R numbers (i think the range of a servo 
        //is 0-180, but you might have to experiment)
        //trayTiltL = trayTiltLeft.getPosition();
        tilt = gate.getPosition();

        //starting position of the drop tray servos
       // trayDropTiltL = trayDropLeft.getPosition();


        // starting position of the balls servo
        // not used

       // ballsPos = balls.getPosition();

        // starting position of the climber servo (also not used)
        //climberPos = climber.getPosition();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */

    @Override
    public void loop() {
        //Put most of the main code here
        
        //This just sets the modes again
       // driveLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
       // driveRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

       // frontRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
       // frontLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

       // liftLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
       // liftRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //climber.setPosition(climberPos);

        // The op mode should only use "write" methods (setPower, setChannelMode, etc) while in
        // WRITE_ONLY mode or SWITCHING_TO_WRITE_MODE
        if (allowedToWrite()) {



            //Gamepad 1 Controls
            //Here, we define some values for controller input    
            //float leftLiftPower = 0;
           // float rightLiftPower = 0;
                float liftPower = 0;
                //float catapultPower = 0;
            double zeroCR = gamepad1.left_trigger;

            if (gamepad1.dpad_down) {
               liftPower = -1;
            }

            else if (gamepad1.dpad_up) {
               liftPower = 1;
            }
            else if (gamepad2.dpad_up) {
               // leftLiftPower = -1;
               // rightLiftPower = 1;
            }
            else if (gamepad2.dpad_down) {
               // leftLiftPower = 1;
                //rightLiftPower = -1;
            }
            //This actually makes the motors run at the values set above
            //motorName.setPower(power) makes things go
            lift.setPower(liftPower);
            //catapult.setPower(catapultPower);
           // liftRight.setPower(rightLiftPower);
            
            //Here, we make values for the drive motor powers 
            //We assign them to the values of the left and right analog sticks
            //This is a quick and easy way to have tank controls
            float rightWheelPower = gamepad1.left_stick_y;
            float leftWheelPower = gamepad1.right_stick_y;

            float frontRightPower = gamepad1.left_stick_y;
            float frontLeftPower = gamepad1.right_stick_y;

            // clip the right/left values so that the values never exceed +/- 1
            // Basically what it says. Motor power can't be bigger than 1
            // So we "clip" it
            rightWheelPower = Range.clip(rightWheelPower, -1, 1);
            leftWheelPower = Range.clip(leftWheelPower, -1, 1);

            frontRightPower = Range.clip(frontRightPower, -1, 1);
            frontLeftPower = Range.clip(frontLeftPower, -1, 1);


            // write the values to the motors
            backRight.setPower(rightWheelPower);
            backLeft.setPower(leftWheelPower);



            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            
            
            //See how the buttons work?
            //If (gamepad#.buttonName)
            //Then--> Do stuff
            //Not too complicated. Just have to figure out button names
            //Most are pretty obvious
            //D-Pad and sticks are a little different, but you can see how
            //They are used
            if (gamepad1.a) {
                launcher.setPower(1);
                RobotLog.w("****DEBUG INIT LINE81 BEGINNING****");

            }

            if (gamepad1.y) {
                launcher.setPower(0);
                RobotLog.w("****DEBUG INIT LINE81 BEGINNING****");
            }

            if (gamepad1.dpad_left) {
                // balls servo button
                // not used
                //Basically, this set the power of a CR servo
                //1 is full speed, 0 is full speed the other way
                //the stopped position was found experimentally
                //which may have been part of the reason we never 
                //actually ended up using CR servos
                //Basically, don't worry about this unless you're using
                //CR servos (continuous rotation)
               // balls.setPosition(1);

            }

            if (gamepad1.dpad_right) {
                //balls servo button
               // balls.setPosition(0);
            }

            if (zeroCR != 0.0) {
                // zero out the cr servo at 0.55
                // not used
               // balls.setPosition(0.55);
            }

            // update the position of the tray manually (not that accurate)
            if (gamepad1.x) {
                //More or less the same stuff as earlier
                tilt += trayDelta;
                gate.setPosition(tilt);
            }

            if (gamepad1.b) {

                tilt -= trayDelta;
                gate.setPosition(tilt);
            }

            //tray drop control
            if (gamepad1.left_bumper) {
                //trayDropTiltL += trayDelta;
                //trayDropTiltR += trayDelta;
            }

            if (gamepad1.right_bumper) {
                //trayDropTiltR -= trayDelta;
                //trayDropTiltL -= trayDelta;
            }

            // clip the position values so that they never exceed 0..1
            //Okay, so I lied. Servo values go from 0 to 1
            //0 is all the way turnt up. 1 is all the way turnt down
            //(up and down are relative)
            //It's the same principle as clipping DC motors. We can't
            //set the servos to too big a value
            tilt = Range.clip(tilt, 0, 1);
            //trayTiltL = Range.clip(trayTiltL, 0, 1);

            //clip the position values so that they never exceed 0..1
           // trayDropTiltR = Range.clip(trayDropTiltR, 0, 1);
            //trayDropTiltL = Range.clip(trayDropTiltL, 0, 1);

            // clip the climber servo position values so that they never exceed 0..1
            //climberPos = Range.clip(climberPos, 0, 1);

            // set tilt servo direction
            //Same as reversing DC motors earlier. Just so things are backwards
            //trayTiltLeft.setDirection(Servo.Direction.FORWARD);
            //servo1.setDirection(Servo.Direction.REVERSE);

            // write position values to the main tray tilt servos
            //This is how you write positions to servos
            //servoName.setPosition(someValue)
            //The value has to be betwen 0 and 1 (inclusive)
           // trayTiltLeft.setPosition(trayTiltL);
           //servo1.setPosition(tilt);

            // set drop servo direction
            //trayDropLeft.setDirection(Servo.Direction.FORWARD);
            //trayDropRight.setDirection(Servo.Direction.REVERSE);

            // write the position values to the drop tray servos
            //trayDropLeft.setPosition(trayDropTiltL);
            //trayDropRight.setPosition(trayDropTiltR);

            // write the position values to the climber servos
            //climber.setPosition(climberPos);



            // we only want to process gamepad2 if someone is using one of it's analog inputs. If you always
            // want to process gamepad2, remove this check

            // possibly throttle the values for motors? (for accuracy)
            
            
            //THIS GETS A LITTLE COMPLICATED
            //We used this setup because we had a shenanigans control scheme
            //Because me and Luke share brain waves. So either use a different 
            //control scheme (one you make yourself, using buttons and setting
            //motor balues) or "hold on to your butts"~dinosaurs everywhere.
            if (gamepad2.atRest() == false) {
                //We only used these controls whent the stuff on controller
                //2 were being used

                double overRide = gamepad2.right_trigger;
                //overRide is what we used to see if I wanted to do something
                //Controller 2 was able to press right trigger and take over controls

                //Most of the next few lines are very simlar to above stuff

                // lift power override
                if (gamepad2.dpad_down) {
                    liftPower = -1;
                }

                if (gamepad2.dpad_up) {
                    liftPower = 1;
                }

                lift.setPower(liftPower);
                //liftRight.setPower(rightLiftPower);

                // if override is activated
                //This stuff only happens oif cont. 2 is taking over
                //(by hitting R2. Or whatever button was assigned as override
                if (overRide != 0.0) {
                    //Look familiar? same code as before, but controlled
                    //by player 2

                    // set drive powers to gamepad 2 values
                    rightWheelPower = gamepad2.left_stick_y;
                    leftWheelPower = gamepad2.right_stick_y;

                    frontRightPower = gamepad2.left_stick_y;
                    frontLeftPower = gamepad2.right_stick_y;

                    // clip the right/left values so that the values never exceed +/- 1
                    rightWheelPower = Range.clip(rightWheelPower, -1, 1);
                    leftWheelPower = Range.clip(leftWheelPower, -1, 1);

                    frontRightPower = Range.clip(frontRightPower, -1, 1);
                    frontLeftPower = Range.clip(frontLeftPower, -1, 1);


                    // write the values to the motors
                    backRight.setPower(rightWheelPower);
                    backLeft.setPower(leftWheelPower);

                    frontRight.setPower(frontRightPower);
                    frontLeft.setPower(frontLeftPower);

                    if (gamepad2.a) {
                        RobotLog.w("****DEBUG INIT LINE81 BEGINNING****");
                        launcher.setPower(1);

                    }

                    if (gamepad2.y) {
                        RobotLog.w("****DEBUG INIT LINE81 BEGINNING****");
                        launcher.setPower(0);
                    }


                    // tray override(s)

                    // update the position of the tray manually (not that accurate)
                    if (gamepad2.x) {
                        tilt += trayDelta;

                    }

                    if (gamepad2.b) {
                        tilt -= trayDelta;

                    }

                    //tray drop control
                    if (gamepad2.left_bumper) {
                        //trayDropTiltL += trayDelta;
                       // trayDropTiltR += trayDelta;
                    }

                    if (gamepad2.right_bumper) {
                       // trayDropTiltR -= trayDelta;
                       // trayDropTiltL -= trayDelta;
                    }

                    // clip the position values so that they never exceed 0..1
                    tilt = Range.clip(tilt, 0, 1);
                    //trayTiltL = Range.clip(trayTiltL, 0, 1);

                    //clip the position values so that they never exceed 0..1
                    //trayDropTiltR = Range.clip(trayDropTiltR, 0, 1);
                   // trayDropTiltL = Range.clip(trayDropTiltL, 0, 1);

                    // clip the climber servo position values so that they never exceed 0..1
                    //climberPos = Range.clip(climberPos, 0, 1);

                    // set tilt servo direction
                    //trayTiltLeft.setDirection(Servo.Direction.FORWARD);
                    //servo1.setDirection(Servo.Direction.REVERSE);

                    // write position values to the main tray tilt servos
                   // trayTiltLeft.setPosition(trayTiltL);
                    //servo1.setPosition(tilt);

                    // set drop servo direction
                    //trayDropLeft.setDirection(Servo.Direction.FORWARD);
                    //trayDropRight.setDirection(Servo.Direction.REVERSE);

                    // write the position values to the drop tray servos
                    //trayDropLeft.setPosition(trayDropTiltL);
                    //trayDropRight.setPosition(trayDropTiltR);

                    // write the position values to the climber servos
                    //climber.setPosition(climberPos);

                } else {
                    // do something else?
                }
            }
        }

    // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
    // It takes time for the hardware to switch, so you can't switch modes within one loop of the
    // op mode. Every 20th loop, this op mode switches to READ_ONLY mode, and gets the current power.

    if(numOpLoops%20==0)
    //This just means if this loop number is divisible by 20. If you 
        //know mod operator, awesome, if not, DDSkogsJr. will tell you it's awesome
    {
        // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
        // before doing a read, and into "write" mode before doing a write. This is because
        // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
        // using the USBDcMotorController, there is no need to switch, because USB can handle reads
        // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
        // This method does nothing on USB devices, but is needed on Nxt devices.

        //rightController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        //leftController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

        //reset loop counter
        numOpLoops = 0;


    }

    // Every 20 loops, switch to read mode so we can read data from the NXT device.
    // Only necessary on NXT devices.
    if(numOpLoops==0)

    {

       // rightController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
       // leftController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
       // frontController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);

        // Update the reads after some loops, when the command has successfully propagated through.
        //telemetry is the readout on the bottom of the phone the controllers
        //run out of. Lets you see some values and stuff, whatever you want
        //just use this first line
        telemetry.addData("ASPECT ", "VALUE");
        //And then add whatever you want with
        //telemetry.addData("Name", value.toString())
        telemetry.addData("DriveRunMode ", backLeft.getMode().toString());
       // telemetry.addData("LiftRunMode ", liftLeft.getMode().toString());

        telemetry.addData("Left Drive ", backLeft.getPower());
        telemetry.addData("Right Drive ", backRight.getPower());

        telemetry.addData("Front Right ", frontRight.getPower());
        telemetry.addData("Front Left ", frontLeft.getPower());

       // telemetry.addData("Lift Left ", liftLeft.getPower());
        //telemetry.addData("Lift Right ", liftRight.getPower());

        telemetry.addData("Left Stick Y ", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y ", gamepad1.right_stick_y);

       // telemetry.addData("Tray Tilt ", trayTiltLeft.getPosition());
        //telemetry.addData("Tray Tilt T ", servoController.getServoPosition(3));
        //telemetry.addData()
        //telemetry.addData("tSet Position ", servo1);

        //telemetry.addData("tDSet Position ", servo1);
       // telemetry.addData("Tray Drop Tilt ", trayDropRight.getPosition());

        //telemetry.addData("Servo Controller gCI ", servoController.getConnectionInfo());

        telemetry.addData("Override ", gamepad2.right_trigger);
        telemetry.addData("catapult", launcher.getPower());
        //rightController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
        //leftController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
    }


    // Update the current devMode
    //devModeR=rightController.getMotorControllerDeviceMode();
    //devModeL=leftController.getMotorControllerDeviceMode();
    //devModeF=frontController.getMotorControllerDeviceMode();

   // rightController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);
   // leftController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

   // frontController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

    numOpLoops++;

    RobotLog.w("LOOP COUNTER: "+numOpLoops);

    }

    // If the device is in either of these two modes, the op mode is allowed to write to the HW.
    private boolean allowedToWrite() {return true;
       // return ((devModeR == DcMotorController.DeviceMode.WRITE_ONLY) && (devModeL == DcMotorController.DeviceMode.WRITE_ONLY) && (devModeF == DcMotorController.DeviceMode.WRITE_ONLY));
    }
    //Hope this helps. If not, email me. Good luck.
    //Anyways. This sandwich is great. It's so delicious and moist.
    //But really. I made a sandwich. And it's so good
    //Peanut butter. On cinnamon bread. 
    //You guys should try it. Except Stackis. He's not worthy.
    //All my luv. And 3/4 of Gr8r's luv. 
    //~Me
}

