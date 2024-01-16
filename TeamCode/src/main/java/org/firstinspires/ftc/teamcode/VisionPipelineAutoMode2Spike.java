package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipelineAutoMode2Spike extends OpenCvPipeline
{
    // Private instance variables

    // Configuration
    private final boolean isScanningForBlue;
    private final boolean isOutputtingChromaChannel;

    // Working variables
    private boolean isFirstFrameProcessed;
    private Mat processedInput;

    private Mat[] region1_chroma, region2_chroma/*, region3_chroma*/;
    private Mat YCrCb = new Mat();
    private Mat chroma = new Mat();
    private Mat outputMat;
    private volatile int avg1, avg2/*, avg3*/;

    // Returning values
    // These are volatile because they may be accessed by another thread while this pipeline is running
    private volatile SidePosition spikePosition;

    // Points
    /*
     * Points which define the spike mark region rectangles, derived from constant values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    /*private final Point region1_pointA = new Point(
            Constants.VISION_REGION1_TOPLEFT_ANCHOR_POINT.x,
            Constants.VISION_REGION1_TOPLEFT_ANCHOR_POINT.y);
    private final Point region1_pointB = new Point(
            Constants.VISION_REGION1_TOPLEFT_ANCHOR_POINT.x + Constants.VISION_SUBREGION_WIDTH,
            Constants.VISION_REGION1_TOPLEFT_ANCHOR_POINT.y + Constants.VISION_SUBREGION_HEIGHT);
    private final Point region2_pointA = new Point(
            Constants.VISION_REGION2_TOPLEFT_ANCHOR_POINT.x,
            Constants.VISION_REGION2_TOPLEFT_ANCHOR_POINT.y);
    private final Point region2_pointB = new Point(
            Constants.VISION_REGION2_TOPLEFT_ANCHOR_POINT.x + Constants.VISION_SUBREGION_WIDTH,
            Constants.VISION_REGION2_TOPLEFT_ANCHOR_POINT.y + Constants.VISION_SUBREGION_HEIGHT);*/
    /*private final Point region3_pointA = new Point(
            Constants.VISION_REGION3_TOPLEFT_ANCHOR_POINT.x,
            Constants.VISION_REGION3_TOPLEFT_ANCHOR_POINT.y);*/
    /*private final Point region3_pointB = new Point(
            Constants.VISION_REGION3_TOPLEFT_ANCHOR_POINT.x + Constants.VISION_REGION_WIDTH,
            Constants.VISION_REGION3_TOPLEFT_ANCHOR_POINT.y + Constants.VISION_REGION_HEIGHT);*/


    public VisionPipelineAutoMode2Spike(boolean isScanningForBlue, boolean isOutputtingChromaChannel) {
        super();
        this.isScanningForBlue = isScanningForBlue;
        this.isOutputtingChromaChannel = isOutputtingChromaChannel;
        isFirstFrameProcessed = false;
        spikePosition = SidePosition.UNDEFINED;
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'chroma'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToChroma(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_chroma = new Mat[Constants.VISION_REGION1_SUBREGION_COUNT];
        populateSubregionSubmatArray(region1_chroma, Constants.VISION_REGION1_CENTER_ANCHOR_POINT);

        region2_chroma = new Mat[Constants.VISION_REGION2_SUBREGION_COUNT];
        populateSubregionSubmatArray(region2_chroma, Constants.VISION_REGION2_CENTER_ANCHOR_POINT);
        //region1_chroma = chroma.submat(new Rect(region1_pointA, region1_pointB));
        //region2_chroma = chroma.submat(new Rect(region2_pointA, region2_pointB));
        //region3_chroma = chroma.submat(new Rect(region3_pointA, region3_pointB));
    }

    // Method to process a camera frame
    /*
     * This will process the first image that the pipeline is given and find the position that the
     * first spike marker is most likely to be in (left, middle, or right) by determining which area
     * in the image has the highest density of the marker's color. It then stores the resulting
     * position (a SpikePosition enum) in the spikePosition variable to be accessed by the public
     * getSpikePosition() method.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * This method runs with every frame, but the spike markers only need to be found once, so
         * we must make sure that this only runs on the first iteration. The results should still be
         * returned in subsequent iterations so that the processed results can be displayed on the
         * control station, but the output of this algorithm should not change after the first frame.
         */
        boolean isFirstFrame = !isFirstFrameProcessed;
        isFirstFrameProcessed = true;

        if (isFirstFrame) {
            /*
             * Overview of what this is doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After the image has been converted to YCrCb, we can extract just the Cr
             * or the Cb channel. The Cr channel (chroma red) shows the red difference
             * of the colors in the image, so it's good for looking for red colors. The
             * Cb channel (chroma blue) shows the blue difference of the colors in the
             * image, so it's good for looking for blue colors.
             *
             * There are two types of objects that need to be detected: red game
             * objects and blue game objects. Because detecting these colors involves
             * searching different chroma channels, this code has two modes for
             * searching either the Cr or the Cb channel.
             *
             * We then take the average pixel value of 2 different regions on a chroma
             * channel, positioned over two of the positions that the spike marker
             * can be. The darkest of the 2 regions is where we assume the spike marker
             * to be, since that means it is the reddest/bluest of the regions. Unless
             * neither is dark enough, in which case we assume the third spike position.
             * This allows the pipeline to work when only two of the spike markers are
             * visible; this is preferable to checking all three because the camera's
             * placement on the robot cannot fully see all three positions at once from
             * the starting position.
             *
             * In this case, the middle and right spike markers are explicitly scanned
             * for; if the spike marker cannot be seen, it is assumed to be in the left
             * position.
             *
             * We also draw blue rectangles on the screen showing where the sample
             * regions are, as well as a green rectangle over the sample region
             * we believe is on top of the spike marker.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the 3 spike markers and
             * be small enough such that only the marker is sampled, not any of the
             * surroundings or the markers on the opposite side of the game field.
             */

            /*
             * Get the chroma channel of the input frame after conversion to YCrCb
             */
            inputToChroma(input);

            /*
             * Pick the channel that will be annotated/output
             */
            if (isOutputtingChromaChannel) {
                outputMat = chroma.clone();
            } else {
                outputMat = input;
            }

            /*
             * Compute the highest average pixel value of each group of
             * subregions. We're taking the average of a single channel
             * buffer, so the value we need is at index 0. We could have
             * also taken the average pixel value of the 3-channel image,
             * and referenced the value at index 2 here.
             */
            avg1 = 0;
            for (Mat subregion : region1_chroma) avg1 = Math.max(avg1, (int) Core.mean(subregion).val[0]);
            avg2 = 0;
            for (Mat subregion : region2_chroma) avg2 = Math.max(avg2, (int) Core.mean(subregion).val[0]);
            //avg1 = (int) Core.mean(region1_chroma).val[0];
            //avg2 = (int) Core.mean(region2_chroma).val[0];
            //avg3 = (int) Core.mean(region3_chroma).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            drawSubregionSubmatArray(region1_chroma, Constants.VISION_REGION1_CENTER_ANCHOR_POINT, outputMat, Constants.BLUE);
            /*Imgproc.rectangle(
                    outputMat, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    Constants.BLUE, // The color the rectangle is drawn in
                    Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            drawSubregionSubmatArray(region2_chroma, Constants.VISION_REGION2_CENTER_ANCHOR_POINT, outputMat, Constants.BLUE);
            /*Imgproc.rectangle(
                    outputMat, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    Constants.BLUE, // The color the rectangle is drawn in
                    Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            /*Imgproc.rectangle(
                    outputMat, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    Constants.BLUE, // The color the rectangle is drawn in
                    Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/


            /*
             * Find the max of the averages
             */
            /*int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);*/
            int max = Math.max(avg1, avg2);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            //if (Math.abs(avg1 - avg2) < Constants.VISION_CHROMA_THRESHOLD) { // Was it from region 3? (the one that is not explicitly seen)
            if (max < Constants.VISION_CHROMA_THRESHOLD) { // Was it from region 3? (the one that is not explicitly seen)
                spikePosition = SidePosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                /*Imgproc.rectangle(
                        outputMat, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        Constants.GREEN, // The color the rectangle is drawn in
                        Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/
            } else if (max == avg1) { // Was it from region 1?
                spikePosition = SidePosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                drawSubregionSubmatArray(region1_chroma, Constants.VISION_REGION1_CENTER_ANCHOR_POINT, outputMat, Constants.GREEN);
                /*Imgproc.rectangle(
                        outputMat, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        Constants.GREEN, // The color the rectangle is drawn in
                        Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/
            } else { // Was it from region 2?
                spikePosition = SidePosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                drawSubregionSubmatArray(region2_chroma, Constants.VISION_REGION2_CENTER_ANCHOR_POINT, outputMat, Constants.GREEN);
                /*Imgproc.rectangle(
                        outputMat, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        Constants.GREEN, // The color the rectangle is drawn in
                        Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/
            }

            processedInput = outputMat.clone();
        }

        /*
         * Render the 'processedInput' buffer to the viewport.
         * This buffer is used rather than the raw input because
         * it contains the exact frame and annotations from the
         * image analysis.
         */

        return processedInput;
    }

    // This function takes the RGB frame, converts to YCrCb, and extracts the appropriate Cr/Cb channel to the 'chroma' variable
    private void inputToChroma(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, chroma, isScanningForBlue ? 2 : 1);
    }

    // This method fills a submat array with centered and evenly spaced out submats for subregions.
    private void populateSubregionSubmatArray(Mat[] submatArray, Point centerAnchorPoint) {
        Point region_topleft_point = new Point(
                centerAnchorPoint.x - ((submatArray.length * Constants.VISION_SUBREGION_WIDTH) + ((submatArray.length - 1) * Constants.VISION_SUBREGION_DISTANCE)) / 2.0,
                centerAnchorPoint.y - Constants.VISION_SUBREGION_HEIGHT / 2.0);

        for (int i = 0; i < submatArray.length; i++) {
            Point topleft_point =  new Point(
                    region_topleft_point.x + (Constants.VISION_SUBREGION_WIDTH + Constants.VISION_SUBREGION_DISTANCE) * i,
                    region_topleft_point.y);

            Point bottomright_point =  new Point(
                    region_topleft_point.x + Constants.VISION_SUBREGION_WIDTH + (Constants.VISION_SUBREGION_WIDTH + Constants.VISION_SUBREGION_DISTANCE) * i,
                    region_topleft_point.y + Constants.VISION_SUBREGION_HEIGHT);

            submatArray[i] = chroma.submat(new Rect(topleft_point, bottomright_point));
        }
    }

    // This method draws a line of centered and evenly spaced out rectangles to represent submats.
    private void drawSubregionSubmatArray(Mat[] submatArray, Point centerAnchorPoint, Mat target, Scalar color) {
        Point region_topleft_point = new Point(
                centerAnchorPoint.x - ((submatArray.length * Constants.VISION_SUBREGION_WIDTH) + ((submatArray.length - 1) * Constants.VISION_SUBREGION_DISTANCE)) / 2.0,
                centerAnchorPoint.y - Constants.VISION_SUBREGION_HEIGHT / 2.0);

        for (int i = 0; i < submatArray.length; i++) {
            Point topleft_point =  new Point(
                    region_topleft_point.x + (Constants.VISION_SUBREGION_WIDTH + Constants.VISION_SUBREGION_DISTANCE) * i,
                    region_topleft_point.y);

            Point bottomright_point =  new Point(
                    region_topleft_point.x + Constants.VISION_SUBREGION_WIDTH + (Constants.VISION_SUBREGION_WIDTH + Constants.VISION_SUBREGION_DISTANCE) * i,
                    region_topleft_point.y + Constants.VISION_SUBREGION_HEIGHT);

            Imgproc.rectangle(
                    target, // Buffer to draw on
                    topleft_point, // First point which defines the rectangle
                    bottomright_point, // Second point which defines the rectangle
                    color, // The color the rectangle is drawn in
                    Constants.VISION_BOX_THICKNESS); // Thickness of the rectangle lines*/
        }
    }

    // Getter methods
    public SidePosition getSpikePosition() {
        return spikePosition;
    }

    public int getChromaAverage1() {
        return avg1;
    }

    public int getChromaAverage2() {
        return avg2;
    }

    /*public int getChromaAverage3() {
        return avg3;
    }*/
}
