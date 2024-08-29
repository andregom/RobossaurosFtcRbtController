package org.firstinspires.ftc.teamcode;


import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpOfc24", group="Linear Opmode")
public class TeleOpOfc24 extends LinearOpMode {

    private DcMotor motoref = null;
    private DcMotor motoret = null;
    private DcMotor motordf = null;
    private DcMotor motordt = null;
    private DcMotor motordir = null; //motor 1 para Subir/Descer o Linear // VISAO DE FRENTE PARA O ROBO
    private DcMotor motoresq = null; //motor 2 para Subir/Descer o Linear // VISAO DE FRENTE PARA O ROBO
    private DcMotor linear = null; //motor para subir o linear
    //
    // private DcMotor pegador = null;  //pegador para subir e descer a treliça
    private DcMotor coletor = null; // motor coletor de pixels
    Servo coletors;
    int position = 0;
    int erro = 0;
    //double powerc = 0;
    double power;
    //int e0 = 200;
    int e1 = 1300;
    int e2 = 2300;
    double kpl = 0.9;

    //int positionc = 100/ex/;
    //double erroc = 0;
    //TouchSensor sensor_toque;
    //TouchSensor toque_tre;
    @Override
    public void runOpMode() {
        //control
        motoref = hardwareMap.get(DcMotor.class, "ef");//PORTA 0 EXPANSION HUB
        motoret = hardwareMap.get(DcMotor.class, "et");//PORTA 3 EXPANSION HUB
        motoresq = hardwareMap.get(DcMotor.class, "ntm");//PORTA 0 EXPANSION HUB
        motordir = hardwareMap.get(DcMotor.class, "mtmm");//PORTA 3 EXPANSION HUB
        motordf = hardwareMap.get(DcMotor.class, "df");//PORTA 3 CONTROL HUB
        motordt = hardwareMap.get(DcMotor.class, "dt");//PORTA 0 CONTROL HUB
        linear = hardwareMap.get(DcMotor.class, "mlinear");//PORTA 1 CONTROL HUB
        coletor = hardwareMap.get(DcMotor.class, "mcoletor");//PORTA 1 CONTROL HUB

        //sensor_toque = hardwareMap.get(TouchSensor.class, "sensor_toque");//PORTA 0:1 ANALOGIC CONTROL
        //coletor = hardwareMap.get(DcMotor.class, "clinear");//PORTA 1 CONTROL HUB
        //toque_tre = hardwareMap.get(TouchSensor.class, "toque_tre");//PORTA 2:3 ANALOGIC CONTROL
        coletors = hardwareMap.get(Servo.class, "coletorss");
        motoref.setDirection(DcMotor.Direction.REVERSE);
        motoret.setDirection(DcMotor.Direction.REVERSE);
        motordf.setDirection(DcMotor.Direction.FORWARD);
        motordt.setDirection(DcMotor.Direction.FORWARD);

        motordir.setDirection(DcMotor.Direction.FORWARD);
        motoresq.setDirection(DcMotor.Direction.REVERSE);
        coletor.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoresq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Init", "Pressionado");
        telemetry.update();

        while (linear.getCurrentPosition() != 0 && motoresq.getCurrentPosition() != 0 && motordir.getCurrentPosition() != 0) {
            idle();
        }

        waitForStart();

        telemetry.addData("Start", "Pressionado");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
//-------------------------------MOBILIDADE------------------------------------------------
            double max;

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double eixo = gamepad1.right_stick_x * 0.8;

            double motoretP = y + x + eixo;
            double motordtP = y - x - eixo;
            double motorefP = y - x + eixo;
            double motordfP = y + x - eixo;

            max = Math.max(Math.abs(motorefP), Math.abs(motordfP));
            max = Math.max(max, Math.abs(motoretP));
            max = Math.max(max, Math.abs(motordtP));

            if (max > 1.0) {
                motorefP /= max;
                motordfP /= max;
                motoretP /= max;
                motordtP /= max;
            }

            motoref.setPower(motorefP);
            motordf.setPower(motordfP);
            motoret.setPower(motoretP);
            motordt.setPower(motordtP);
//------------------------------------------------------------------------------------------------------------------
            motordir.setPower(gamepad2.right_stick_y);
            motoresq.setPower(gamepad2.right_stick_y);
//------------------------------------------------------------------------------------------------------------------
            /*if (sensor_toque.isPressed()){
                linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Encoder", "Resetado");
                telemetry.update();}*/
            if (gamepad2.x) {//Primeiro Estágio do Linear
                position = linear.getCurrentPosition();
                erro = position - 1300;
                linear.setTargetPosition(e1);
                linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
                coletors.setPosition(-1);
                telemetry.addData("Estagio", "1");
            } else if (gamepad2.y) {//Segundo Estágio do Linear
                position = linear.getCurrentPosition();
                erro = position - 2300;
                linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linear.setTargetPosition(e2);
                sleep(1150);
                coletors.setPosition(-1);
                telemetry.addData("Estagio", "2");
            } else if (gamepad2.a) {//Descer Linear até chegar ao sensor de toque
                coletors.setPosition(1);
                sleep(1500);
                linear.setTargetPosition(0);
                linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linear.setPower(1);
                telemetry.addData("Estagio", "0");
            } else if (gamepad2.left_bumper) {
                coletors.setPosition(1);
            } else if (gamepad2.right_bumper) {
                coletors.setPosition(1);
            }
            //-----------------------------------------------------pendurar treliça-------------------------------------
            motoresq.setPower(gamepad2.left_stick_y);
            motordir.setPower(gamepad2.right_stick_y);
            //-------------------------------------------------COLETADOR DE PIXELS--------------------------------------------------------------
            coletor.setPower(gamepad2.right_trigger);//COLETAR PIXELS COM GATILHO DIREITO(ESTEIRA)
            coletor.setPower(-gamepad2.left_trigger);
//--------------------------------------------PROPORCIONAL LINEAR PIXEL----------------------------------------------------------------------
            power = erro * kpl;
            if (linear.isBusy()) {
                linear.setPower(power);
            }
        }
    }
}