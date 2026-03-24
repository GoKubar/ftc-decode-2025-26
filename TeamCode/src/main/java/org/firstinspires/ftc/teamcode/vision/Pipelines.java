package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.ColorPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.LocalizationPipeline;
import org.firstinspires.ftc.teamcode.vision.pipelines.RelocalizationPipeline;

import java.util.HashMap;
import java.util.Map;

public enum Pipelines {
    APRIL_TAG(AprilTagPipeline::new),
    COLOR(ColorPipeline::new),
    LOCALIZATION(LocalizationPipeline::new),
    RELOCALIZATION(RelocalizationPipeline::new),
    NONE;

    private final VisionPipelineSupplier supplier;
    private static final Map<VisionPipeline, Pipelines> pipelineEnumMap = new HashMap<>();

    Pipelines(VisionPipelineSupplier supplier) {
        this.supplier = supplier;
    }

    Pipelines() {
        this.supplier = telemetry -> null;
    }

    public VisionPipeline build(Telemetry telemetry) {
        VisionPipeline pipeline = supplier.get(telemetry);
        if (pipeline != null) {
            pipelineEnumMap.put(pipeline, this);
        }
        return pipeline;
    }

    public static Pipelines get(VisionPipeline pipeline) {
        return pipelineEnumMap.getOrDefault(pipeline, NONE);
    }

    public static Pipelines[] getActivePipelines() {
        return new Pipelines[] { APRIL_TAG, COLOR, LOCALIZATION, RELOCALIZATION };
    }
}
