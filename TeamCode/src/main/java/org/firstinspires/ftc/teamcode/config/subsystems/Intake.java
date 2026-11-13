package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** Two-motor transfer (left + right, driven together) and the left/right gates in front of the flywheel. */
@Config
public class Intake {
    public static double intakePower = 1;
    /** Transfer power while feeding the flywheel. */
    public static double shootPower = .7;
    public static double outtakePower = -1;
    public static double slowOuttakePower = -.7;

    public static double leftOpen = .099, leftClosed = .229;
    public static double rightOpen = .958, rightClosed = .731;
    /** Dashboard override: hold each gate at its manual position. */
    public static boolean manual = false;
    public static double manualLeftPos = .5, manualRightPos = .5;

    public enum TransferState { OFF, INTAKE, SHOOT, OUTTAKE, SLOW_OUTTAKE }
    public enum GateState { OPEN, CLOSED }

    private final MultipleTelemetry telemetry;
    private final Servo gateL, gateR;
    private final DcMotorEx transferL, transferR;

    public TransferState currentTransfer = TransferState.OFF;
    public GateState currentGate = GateState.CLOSED;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gateL = hardwareMap.get(Servo.class, "cs5");
        gateR = hardwareMap.get(Servo.class, "es5");
        transferL = hardwareMap.get(DcMotorEx.class, "em3");
        transferR = hardwareMap.get(DcMotorEx.class, "em0");

        transferL.setDirection(DcMotorSimple.Direction.REVERSE);
        transferR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void init() {
        setTransferState(TransferState.OFF);
        setPower(0);
        setGateState(GateState.CLOSED);
        gateL.setPosition(leftClosed);
        gateR.setPosition(rightClosed);
    }

    public void setTransferState(TransferState state) { currentTransfer = state; }
    public void setGateState(GateState state) { currentGate = state; }

    public void periodic() {
        switch (currentTransfer) {
            case OFF:          setPower(0); break;
            case INTAKE:       setPower(intakePower); break;
            case SHOOT:        setPower(shootPower); break;
            case OUTTAKE:      setPower(outtakePower); break;
            case SLOW_OUTTAKE: setPower(slowOuttakePower); break;
        }

        boolean open = currentGate == GateState.OPEN;
        gateL.setPosition(manual ? manualLeftPos : open ? leftOpen : leftClosed);
        gateR.setPosition(manual ? manualRightPos : open ? rightOpen : rightClosed);

        if (showTelemetry) {
            telemetry.addData("Transfer state", currentTransfer);
            telemetry.addData("Gate state", currentGate);
            telemetry.addData("Transfer L amps", transferL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Transfer R amps", transferR.getCurrent(CurrentUnit.AMPS));
        }
    }

    private void setPower(double power) {
        transferL.setPower(power);
        transferR.setPower(power);
    }
}
