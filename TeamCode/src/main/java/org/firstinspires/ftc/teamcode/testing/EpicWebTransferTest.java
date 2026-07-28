package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.DECODE_subsystems.Intake;

import java.io.*;
import java.net.*;

public class EpicWebTransferTest extends TestingOpMode {

    private volatile int variable1 = 0;
    private volatile int variable2 = 0;
    private Intake intake;
    @Override
    public void runOpMode() {
        makeTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        // Create a socket server that listens for incoming data
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    ServerSocket serverSocket = new ServerSocket(8888);
                    while (!Thread.currentThread().isInterrupted()) {
                        Socket clientSocket = serverSocket.accept();
                        BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                        String receivedData = in.readLine();
                        String[] variables = receivedData.split(",");
                        variable1 = Integer.parseInt(variables[0]);
                        variable2 = Integer.parseInt(variables[1]);
                        intake.moveSlides(variable1);
                        intake.moveArm(variable2);
                        telemetry.addData("Variable1", variable1);
                        telemetry.addData("Variable2", variable2);
                        telemetry.update();
                        clientSocket.close();
                    }
                } catch (IOException e) {
                    telemetry.addData("Error", e.getMessage());
                    telemetry.update();
                }
            }
        }).start();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Robot logic using updated variables
            intake.update();
            telemetry.addData("Running Variable1", variable1);
            telemetry.addData("Running Variable2", variable2);
            telemetry.update();
        }
    }
}
