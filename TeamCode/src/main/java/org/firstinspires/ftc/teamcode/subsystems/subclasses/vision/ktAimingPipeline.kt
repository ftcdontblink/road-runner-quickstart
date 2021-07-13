package org.firstinspires.ftc.teamcode.subsystems.subclasses.vision

import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.teamcode.subsystems.subclasses.vision.CalibrationParameters
import org.firstinspires.ftc.teamcode.subsystems.subclasses.vision.VisionProcessor
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import java.lang.Math.atan2

class KtAimingPipeline : VisionProcessor<KtAimingPipeline.AimingOutput>() {
    enum class Color {
        BLUE, RED
    }

    data class AimingOutput(val captureTimeNanos: Long, val highGoalAngle: Double, val powershotAngles: DoubleArray) {
        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (javaClass != other?.javaClass) return false

            other as AimingOutput

            if (captureTimeNanos != other.captureTimeNanos) return false
            if (highGoalAngle != other.highGoalAngle) return false
            if (!powershotAngles.contentEquals(other.powershotAngles)) return false

            return true
        }

        override fun hashCode(): Int {
            var result = captureTimeNanos.hashCode()
            result = 31 * result + highGoalAngle.hashCode()
            result = 31 * result + powershotAngles.contentHashCode()
            return result
        }
    }

    companion object {
        @JvmField
        var CALIB_PARAMS = CalibrationParameters(481.5, 483.5, 310.9, 186.0,
                0.0504173, -0.16018, -0.00134816, 0.00153469, 0.0942480)

        @JvmField
        var HSV_LOWER_BLUE = Scalar(100.0, 100.0, 100.0)
        @JvmField
        var HSV_UPPER_BLUE = Scalar(120.0, 256.0, 256.0)
        @JvmField
        var HSV_LOWER_RED = Scalar(170.0, 100.0, 100.0)
        @JvmField
        var HSV_UPPER_RED = Scalar(10.0, 256.0, 256.0)

        // tl, bl, tr, br
        var BLUE_SLOT_POINTS = MatOfPoint3f( Point3(72.0, 42.91, 38.54), Point3(71.7, 43.04, 33.16),
                Point3(72.0, 27.09, 38.54), Point3(71.7, 26.96, 33.16))

        var BLUE_POWERSHOT_POINTS = MatOfPoint3f(Point3(71.0, 18.75, 30.0),
                Point3(71.0, 11.25, 30.0), Point3(71.0, 3.75, 30.0))

        var BLUE_HIGH_GOAL_POINT = Point3(72.0, 35.0, 36.0)

        var RED_SLOT_POINTS = MatOfPoint3f( Point3(72.0, -27.09, 38.54), Point3(71.7, -26.96, 33.16),
                Point3(72.0, -42.91, 38.54), Point3(71.7, -43.04, 33.16))

        var RED_POWERSHOT_POINTS = MatOfPoint3f(Point3(71.0, -18.75, 30.0),
                Point3(71.0, -11.25, 30.0), Point3(71.0, -3.75, 30.0))

        var RED_HIGH_GOAL_POINT = Point3(72.0, -35.0, 36.0)

        @JvmField
        var MIN_GOAL_WIDTH = 60.0

        @JvmField
        var MIN_SLOT_WIDTH = 20.0
        @JvmField
        var MAX_SLOT_WIDTH = 250.0

        // width / height ratio
        @JvmField
        var MIN_SLOT_RATIO = 1.5
        @JvmField
        var MAX_SLOT_RATIO = 5.0

        // static so they're controllable through dashboard
        @JvmField
        var stageNum = 0
    }

    private val filteredMat = Mat()
    private val ycrcbMat = Mat()
    private val cColorMat = Mat()
    private val hsvMat = Mat()
    private val rangeMat = Mat()
    private val range1Mat = Mat()
    private val range2Mat = Mat()
    private val denoisedRangeMat = Mat()
    private val goalContourMat = Mat()
    private var convexGoalMat = Mat()
    private var slotMat = Mat()
    private var denoisedSlotMat = Mat()
    private var selectedSlotEdgeMat = Mat()
    private val slotContourMat = Mat()
    private var selectedSlotMat = Mat()
    private var dilatedSelectedSlotMat = Mat()
    private val finalMat = Mat()

    private val hierarchy = Mat()
    private val goalContours: MutableList<MatOfPoint> = ArrayList()

    private val hullIndices = MatOfInt()

    private val slotContours: MutableList<MatOfPoint> = ArrayList()

    private val contour2f = MatOfPoint2f()
    private val approx2f = MatOfPoint2f()
    private val approx = MatOfPoint()

    private val corners = MatOfPoint()
    private val corners2f = MatOfPoint2f()

    private val rvec = Mat()
    private val tvec = Mat()

    private val rotation = Mat()
    private val rotationInv = Mat()
    private val actualPos = Mat()

    private val wideElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(5.0, 3.0))
    private val normalElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(3.0, 3.0))

    var color = Color.BLUE

    override fun onViewportTapped() {
        stageNum++
    }

    override fun frameToResult(input: Mat, captureTimeNanos: Long): VisionProcessorResult<AimingOutput> {
//        val captureTimeNanos = System.nanoTime() - ((1.0 / 30.0) * 1.0e9).toLong()
        Core.multiply(input, Scalar(0.3, 0.3, 0.3), finalMat)
        input.copyTo(goalContourMat)
        input.copyTo(slotContourMat)

        val camera = CALIB_PARAMS.cameraMatrix
        val distCoeffs = CALIB_PARAMS.distCoeffs

        var output: AimingOutput? = null

        if (convexGoalMat.size() != input.size()) {
            convexGoalMat.release()
            convexGoalMat = Mat.zeros(input.size(), CvType.CV_8UC1)
        } else {
            convexGoalMat.setTo(Scalar(0.0))
        }

        if (selectedSlotMat.size() != input.size()) {
            selectedSlotMat.release()
            selectedSlotMat = Mat.zeros(input.size(), CvType.CV_8UC1)
        } else {
            selectedSlotMat.setTo(Scalar(0.0))
        }

        selectedSlotEdgeMat.setTo(Scalar(0.0))

        // initial noise reduction, bilateral is good for noise reduction while preserving boundaries
//        Imgproc.bilateralFilter(input, filteredMat, 5, 100, 100);
        input.copyTo(filteredMat)

        // rgb -> ycrcb
        Imgproc.cvtColor(filteredMat, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        // rgb -> hsv
        Imgproc.cvtColor(filteredMat, hsvMat, Imgproc.COLOR_RGB2HSV)

        // extract cr or cb channel based on color
        Core.extractChannel(ycrcbMat, cColorMat, if (color == Color.BLUE) 2 else 1)
        val hsvLower = if (color == Color.BLUE) HSV_LOWER_BLUE else HSV_LOWER_RED
        val hsvUpper = if (color == Color.BLUE) HSV_UPPER_BLUE else HSV_UPPER_RED

        // hsv filtering
        if (hsvLower.`val`[0] > hsvUpper.`val`[0]) {
            // this "wraps" the hue range (channel 0) into a two part range on both extremes of the h spectrum
            Core.inRange(hsvMat, hsvLower, Scalar(181.0, hsvUpper.`val`[1], hsvUpper.`val`[2]), range1Mat)
            Core.inRange(hsvMat, Scalar(0.0, hsvLower.`val`[1], hsvLower.`val`[2]), hsvUpper, range2Mat)
            Core.bitwise_or(range1Mat, range2Mat, rangeMat)
        } else {
            Core.inRange(hsvMat, hsvLower, hsvUpper, rangeMat)
        }

        // erode + dilate remove small areas and fill gaps
        Imgproc.dilate(rangeMat, denoisedRangeMat, normalElement, Point(-1.0, -1.0), 2)
        Imgproc.erode(denoisedRangeMat, denoisedRangeMat, normalElement, Point(-1.0, -1.0), 6)
        Imgproc.dilate(denoisedRangeMat, denoisedRangeMat, normalElement, Point(-1.0, -1.0), 4)

        // finds contours in the filtered mat
        Imgproc.findContours(denoisedRangeMat, goalContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(goalContourMat, goalContours, -1, Scalar(0.0, 255.0, 0.0), 6)

        goalContours.forEach { contour ->
            // contour must pass a minimum width check
            val rect = Imgproc.boundingRect(contour)
            if (rect.width < MIN_GOAL_WIDTH) return@forEach

            // convert contour into convex contour
            Imgproc.convexHull(contour, hullIndices)
            val contourArray = contour.toArray()
            contour.fromList(hullIndices.toArray().map { contourArray[it] })

            Imgproc.drawContours(goalContourMat, listOf(contour), -1, Scalar(255.0, 255.0, 0.0), 3)

            // fill the convex contour
            Imgproc.fillConvexPoly(convexGoalMat, contour, Scalar(255.0))
        }

        Core.copyTo(input, finalMat, convexGoalMat)

        Core.subtract(convexGoalMat, denoisedRangeMat, slotMat)

        Imgproc.erode(slotMat, denoisedSlotMat, wideElement, Point(-1.0, -1.0), 2)
        Imgproc.dilate(denoisedSlotMat, denoisedSlotMat, wideElement, Point(-1.0, -1.0), 2)

        // draw slot candidate contours
        Imgproc.findContours(denoisedSlotMat, slotContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(slotContourMat, slotContours, -1, Scalar(0.0, 255.0, 0.0), 6)


        slotContours.forEach { contour ->
            // contour must pass a width an ratio checks
            val rect = Imgproc.boundingRect(contour)
            val ratio = rect.width.toDouble() / rect.height.toDouble()
            if (rect.width < MIN_SLOT_WIDTH || rect.width > MAX_SLOT_WIDTH
                    || ratio < MIN_SLOT_RATIO || ratio > MAX_SLOT_RATIO) return@forEach

            // convert contour into approximated contour
            contour2f.fromArray(*contour.toArray())
            val arcLength = Imgproc.arcLength(contour2f, true)
            Imgproc.approxPolyDP(contour2f, approx2f, arcLength * 0.02, true)
            approx.fromArray(*approx2f.toArray())

            Imgproc.drawContours(slotContourMat, listOf(approx), -1, Scalar(255.0, 255.0, 0.0), 3)

            // approximation should be a quadrilateral
            if (approx.toArray().size != 4) return@forEach

            // fill the final goal contour
            Imgproc.drawContours(selectedSlotMat, listOf(contour), -1, Scalar(255.0), -1)

            Imgproc.dilate(selectedSlotMat, dilatedSelectedSlotMat, normalElement, Point(-1.0, -1.0), 3)
            Core.bitwise_and(dilatedSelectedSlotMat, denoisedRangeMat, selectedSlotEdgeMat)

            Imgproc.dilate(selectedSlotEdgeMat, selectedSlotEdgeMat, normalElement, Point(-1.0, -1.0), 3)

            // find goal slot corners using the selected slot's edge as a mask
            Imgproc.goodFeaturesToTrack(cColorMat, corners, 4, 0.02, 6.0,
                    selectedSlotEdgeMat, 5, 5, false, 0.04)

            // must have four corners on the slot :P
            if (corners.rows() != 4) return@forEach

            // refine the detected corner coordinates
            corners2f.fromArray(*corners.toArray())
            Imgproc.cornerSubPix(
                    cColorMat, corners2f, Size(9.0, 9.0), Size(-1.0, -1.0),
                    TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 40, 0.001)
            )

            val points = corners2f.toArray()

            // sort in the order defined for the slot coordinates (first left to right, then vertical comparisons)
            points.sortBy { it.x }

            if (points[1].y < points[0].y) {
                val temp = points[0]
                points[0] = points[1]
                points[1] = temp
            }

            if (points[3].y < points[2].y) {
                val temp = points[2]
                points[2] = points[3]
                points[3] = temp
            }

            corners2f.fromArray(*points)

            val (slotPoints, powershotPoints) = if (color == Color.BLUE) Pair(BLUE_SLOT_POINTS, BLUE_POWERSHOT_POINTS)
            else Pair(RED_SLOT_POINTS, RED_POWERSHOT_POINTS)
            Calib3d.solvePnP(slotPoints, corners2f, camera, distCoeffs, rvec, tvec, false)

            Calib3d.Rodrigues(rvec, rotation)
            Core.invert(rotation, rotationInv)
            Core.gemm(rotationInv, tvec, -1.0, Mat(), 0.0, actualPos)

            val buff = DoubleArray(3)
            actualPos.get(0, 0, buff)
            val cameraPos = Point3(buff)

            // uses first two components of R^(-1)*k in atan2 (k is the unit vector in the z direction)
            // this is because +z is forward (out of the lens) in camera-space so we invert the rotation to get this
            // direction in field space and calculate its angle
            val baseAngle = atan2(rotationInv.get(1, 2)[0], rotationInv.get(0, 2)[0])
            val powershotAngles = powershotPoints.toList()
                    .map { Angle.normDelta(atan2(it.y - cameraPos.y, it.x - cameraPos.x) - baseAngle) }
            val highGoalPoint = if (color == Color.BLUE) BLUE_HIGH_GOAL_POINT else RED_HIGH_GOAL_POINT
            val highGoalAngle = Angle.normDelta(atan2(highGoalPoint.y - cameraPos.y, highGoalPoint.x - cameraPos.x) - baseAngle)

            output = AimingOutput(captureTimeNanos + 1, highGoalAngle, powershotAngles.toDoubleArray())

            val impts = MatOfPoint2f()

            Calib3d.projectPoints(powershotPoints, rvec, tvec, camera, distCoeffs, impts)

//            Imgproc.line(finalMat, points[0], impts.toArray()[0], Scalar(255.0, 0.0, 0.0), 3)
//            Imgproc.line(finalMat, points[0], impts.toArray()[1], Scalar(0.0, 255.0, 0.0), 3)
//            Imgproc.line(finalMat, points[0], impts.toArray()[2], Scalar(0.0, 0.0, 255.0), 3)

            points.forEach { Imgproc.circle(finalMat, it, 5, Scalar(0.0, 255.0, 0.0), 1) }
            impts.toList().forEach { Imgproc.circle(finalMat, it, 5, Scalar(255.0, 255.0, 0.0), 1) }
        }

        goalContours.clear()
        slotContours.clear()

        val debugMats = arrayOf(
                finalMat, filteredMat, /* yMat, crMat, cbMat, */ denoisedRangeMat, goalContourMat,
                convexGoalMat, denoisedSlotMat, slotContourMat, selectedSlotMat, selectedSlotEdgeMat
        )
        val displayMat = debugMats[stageNum % debugMats.size]
        return VisionProcessorResult(output, displayMat)
    }
}