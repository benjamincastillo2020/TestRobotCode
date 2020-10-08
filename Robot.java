package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Robot {

    ElapsedTime runtime = new ElapsedTime();
    DcMotor    Front_Left;
    DcMotor    Front_Right;
    DcMotor    Back_Left;
    DcMotor    Back_Right;
    DcMotor    Lift_Drive ;
    CRServo    BP1;
    CRServo    BP2;
    Servo      Latch1;
    Servo      Latch2;
    public OpMode opMode;
    public LinearOpMode LinearOpMode;
    public HardwareMap hardwareMap;

    public Robot(LinearOpMode l) {
        LinearOpMode = l;
        this.hardwareMap = LinearOpMode.hardwareMap;

    }
    public Robot(OpMode o) {
        opMode = o;
        this.hardwareMap = opMode.hardwareMap;

    }
    public CRServo[] CRServoArray = {BP1, BP2};
    public Servo[] ServoArray = {Latch1, Latch2};
    public DcMotor[] MotorArray = {Front_Left, Front_Right, Back_Left, Back_Right, Lift_Drive };

    public void attachCRServos(CRServo[] servo) {
        for (int i = 0; i < servo.length; i++){
            CRServo servo1 = servo[i];
            if (servo1.getDeviceName() == null) {
                i++;
            }
            else {
                servo1 = hardwareMap.get(CRServo.class, servo1.getDeviceName());
            }
        }
    }

    public void attachServos(Servo[] servo) {
        for (int i = 0; i < servo.length; i++) {
            Servo servo1 = servo[i];
            servo1 = hardwareMap.get(Servo.class, servo1.getDeviceName());
        }
    }

    public void attachMotors(DcMotor[] motor) {

        for (int i = 0; i < motor.length; i++){
            DcMotor motor1 = motor[i];
            motor1  = hardwareMap.get(DcMotor.class, motor1.getDeviceName());
        }
    }



    public void attachAll(){
        attachCRServos(CRServoArray);
        attachMotors(MotorArray);
        attachServos(ServoArray);

    }

    public void MecFoward (int distance, double power){
        distance = distance * 1120;

        power = power / 100;
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left.setTargetPosition(distance);
        Front_Right.setTargetPosition(distance);
        Back_Left.setTargetPosition(distance);
        Back_Right.setTargetPosition(distance);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left.setPower(power);
        Front_Right.setPower(power);
        Back_Left.setPower(power);
        Back_Right.setPower(power);
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }
    public void MecBack (int distance, double power){
        distance = distance * 1120;

        power = power / 100;
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left.setTargetPosition(distance);
        Front_Right.setTargetPosition(distance);
        Back_Left.setTargetPosition(distance);
        Back_Right.setTargetPosition(distance);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left.setPower(-power);
        Front_Right.setPower(-power);
        Back_Left.setPower(-power);
        Back_Right.setPower(-power);
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }
    public void mechRightSlide(int distance, double power){
        distance = distance * 1120;

        power = power / 100;
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left.setTargetPosition(distance);
        Front_Right.setTargetPosition(distance);
        Back_Left.setTargetPosition(distance);
        Back_Right.setTargetPosition(distance);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left.setPower(power);
        Front_Right.setPower(-power);
        Back_Left.setPower(-power);
        Back_Right.setPower(power);
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void mechLeftSlide(int distance, double power){
        distance = distance * 1120;

        power = power / 100;
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left.setTargetPosition(distance);
        Front_Right.setTargetPosition(distance);
        Back_Left.setTargetPosition(-distance);
        Back_Right.setTargetPosition(-distance);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left.setPower(-power);
        Front_Right.setPower(power);
        Back_Left.setPower(power);
        Back_Right.setPower(-power);
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void MechRotate(int distance, double power, boolean direction){
        distance = distance * 1120;
        if (direction == true) {
            power = power / 100;
        }
        else {
            power = (power / 100) * -1;
        }

        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Left.setTargetPosition(-distance);
        Front_Right.setTargetPosition(-distance);
        Back_Left.setTargetPosition(distance);
        Back_Right.setTargetPosition(distance);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left.setPower(power);
        Front_Right.setPower(-power);
        Back_Left.setPower(power);
        Back_Right.setPower(-power);
        Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
