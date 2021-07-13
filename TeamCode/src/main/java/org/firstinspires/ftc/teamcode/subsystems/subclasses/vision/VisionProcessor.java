package org.firstinspires.ftc.teamcode.subsystems.subclasses.vision;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

public abstract class VisionProcessor<T> extends TimestampedOpenCvPipeline {

    // NOTE: downstream vision processors will not get access to onViewportTapped events
    public static class MultipleProcessorPipeline extends TimestampedOpenCvPipeline {
        // NOTE: tentatively removing Collections.synchronizedList because everything should be properly
        // synchronized without it now
        private final List<VisionProcessor<?>> processors = new ArrayList<>();
        private int displayIdx = 0;

        public void addProcessor(VisionProcessor<?> processor) {
            synchronized (processors) {
                processors.add(processor);
            }
        }

        public void removeProcessor(VisionProcessor<?> processor) {
            synchronized (processors) {
                int idx = processors.indexOf(processor);
                processors.remove(processor);
                synchronized (this) {
                    if (idx >= displayIdx) {
                        displayIdx = Math.min(Math.max(displayIdx - 1, 0), processors.size());
                    }
                }
            }
        }

        @Override
        public void onViewportTapped() {
            synchronized (this) {
                displayIdx++;
                displayIdx %= processors.size();
            }
        }

        public void init(Mat mat) {
            synchronized (processors) {
                for (VisionProcessor<?> processor : processors) {
                    Mat internalFrame = processor.getInternalFrame();
                    mat.copyTo(internalFrame);
                    processor.init(internalFrame);
                }
            }
        }

        @Override
        public Mat processFrame(Mat input, long captureTimeNanos) {
            Mat returnMat = input;
            synchronized (processors) {
                for (int i = 0; i < processors.size(); i++) {
                    VisionProcessor<?> processor = processors.get(i);

                    Mat internalFrame = processor.getInternalFrame();
                    input.copyTo(internalFrame);
                    Mat outMat = processor.processFrame(internalFrame, captureTimeNanos);

                    synchronized (this) {
                        if (i == displayIdx) {
                            returnMat = outMat;
                        }
                    }
                }
            }
            return returnMat;
        }
    }

    public static class VisionProcessorResult<K> {
        private final K output;
        private final Mat viewportMat;

        public VisionProcessorResult(K output, Mat viewportMat) {
            this.output = output;
            this.viewportMat = viewportMat;
        }

        public K getOutput() {
            return output;
        }

        public Mat getViewportMat() {
            return viewportMat;
        }
    }

    private T lastOutput;
    private Mat internalFrame = new Mat();

    public Mat processFrame(Mat input, long captureTimeNanos) {
        VisionProcessorResult<T> result = frameToResult(input, captureTimeNanos);
        synchronized (this) {
            lastOutput = result.getOutput();
        }
        return result.getViewportMat();
    }

    public abstract VisionProcessorResult<T> frameToResult(Mat input, long captureTimeNanos);

    public T getLastOutput() {
        synchronized (this) {
            return lastOutput;
        }
    }

    private Mat getInternalFrame() {
        return internalFrame;
    }
}