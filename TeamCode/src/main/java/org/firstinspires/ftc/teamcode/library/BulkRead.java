package org.firstinspires.ftc.teamcode.library;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import kotlin.text.CharDirectionality;

public class BulkRead {
    private List<LynxModule> allHubs;
    public BulkRead(HardwareMap hardwareMap){
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void clearCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Re-asserts MANUAL bulk caching on every hub.
     *
     * MecanumDrive's constructor forces every hub to AUTO (MecanumDrive.java), so any opmode that
     * builds a BulkRead *before* a MecanumDrive/MecaTank silently ends up in AUTO, and clearCache()
     * stops being the thing that controls the read cycle. Under AUTO the hub re-issues a full bulk
     * read whenever a register is read a second time in the same cycle, which costs an extra
     * round trip per repeated read.
     *
     * Call this once after all drivetrain objects are constructed to take back control.
     */
    public void setManual(){
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public class updateAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            clearCache();
            return true;
        }
    }
    public Action update(){
        return new updateAction();
    }
}

