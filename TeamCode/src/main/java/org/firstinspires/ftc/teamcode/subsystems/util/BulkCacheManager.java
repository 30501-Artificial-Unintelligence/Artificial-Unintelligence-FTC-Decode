package org.firstinspires.ftc.teamcode.subsystems.util;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public final class BulkCacheManager {
    private final List<LynxModule> hubs;

    public BulkCacheManager(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // Optional: only if you ever want to force a refresh mid-loop
    public void clearNow() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}

