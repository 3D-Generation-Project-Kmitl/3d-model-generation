/*
 * Copyright 2021 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.ar.core.codelab.rawdepth;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.opengl.Matrix;
import android.util.Log;

import com.google.ar.core.Anchor;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Coordinates2d;
import com.google.ar.core.Frame;
import com.google.ar.core.Plane;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.core.exceptions.UnavailableApkTooOldException;
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;
import com.google.ar.core.exceptions.UnavailableSdkTooOldException;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.nio.charset.Charset;
import java.util.Collection;

/**
 * Converts depth data from ARCore depth images to 3D pointclouds. Points are added by calling the
 * Raw Depth API, and reprojected into 3D space.
 */
public class DepthData {
    public static final int FLOATS_PER_POINT = 6; // X,Y,Z,R,G,B.
    public static byte[] rawDepthData;
    public static byte[] rawDepthPointCloudData;

    public static FloatBuffer create(Frame frame, Anchor cameraPoseAnchor) {
        try {

            Image depthImage = frame.acquireRawDepthImage();
            Image cameraImage = frame.acquireCameraImage();
            Image confidenceImage = frame.acquireRawDepthConfidenceImage();

            //save depth data
            saveDepthData(depthImage);

            // To transform 2D depth pixels into 3D points we retrieve the intrinsic camera parameters
            // corresponding to the depth image. See more information about the depth values at
            // https://developers.google.com/ar/develop/java/depth/overview#understand-depth-values.

            CameraIntrinsics intrinsics = frame.getCamera().getTextureIntrinsics();

            float[] modelMatrix = new float[16];
            cameraPoseAnchor.getPose().toMatrix(modelMatrix, 0);


            final FloatBuffer rawDepthPoints = convertRawDepthImagesTo3dPointBuffer(
                    cameraImage,depthImage, confidenceImage, intrinsics, modelMatrix);

            rawDepthPoints.rewind();
            //save point cloud data
            savePointCloudData(rawDepthPoints);


            System.out.println("depth value: "+Float.toString(getCentimetersDepth(depthImage, 80, 45))+" confidence value: "+Float.toString(getConfidenceDepth(confidenceImage, 80, 45))
            );
            cameraImage.close();
            depthImage.close();
            confidenceImage.close();


            return rawDepthPoints;
        } catch (NotYetAvailableException e) {
            // This normally means that depth data is not available yet. This is normal so we will not
            // spam the logcat with this.
        }
        return null;
    }
    private static void saveDepthData(Image depthImage){
        Image.Plane plane = depthImage.getPlanes()[0];
        int format = depthImage.getFormat();
        ByteBuffer buffer = plane.getBuffer().order(ByteOrder.nativeOrder());
        System.out.println("rawDepthImage");
        int count=0;
        while (buffer.hasRemaining()) {
            System.out.print(buffer.get()+" ");
            count+=1;
            if(count==5){
                break;
            }
        }
        System.out.println();
        System.out.println("rawDepthImage shape");
        rawDepthData = new byte[buffer.remaining()];
        buffer.get(rawDepthData);
    }

    private static void savePointCloudData(FloatBuffer rawDepthPoints){
        System.out.println("rawDepthPointCoulds size: "+rawDepthPoints.remaining());
        int count=0;
        System.out.println("rawDepthPointCoulds");

        while (rawDepthPoints.hasRemaining()) {
            System.out.print(rawDepthPoints.get()+" ");
            count+=1;
            if(count==5){
                break;
            }
        }
        System.out.println();
        System.out.println("rawDepthPoints byte_order "+rawDepthPoints.order());
        if(rawDepthPointCloudData==null){
            rawDepthPointCloudData=convertFloatBufferToByteArray(rawDepthPoints);
        }else {
            byte[] newRawDepthBytes = convertFloatBufferToByteArray(rawDepthPoints);
            byte[] resultByteArray = new byte[rawDepthPointCloudData.length + newRawDepthBytes.length];
            System.arraycopy(rawDepthPointCloudData, 0, resultByteArray, 0, rawDepthPointCloudData.length);
            System.arraycopy(newRawDepthBytes, 0, resultByteArray, rawDepthPointCloudData.length, newRawDepthBytes.length);
            rawDepthPointCloudData = resultByteArray;
        }
    }
    /** Applies camera intrinsics to convert depth image into a 3D pointcloud. */
    private static FloatBuffer convertRawDepthImagesTo3dPointBuffer(
            Image cameraImage,Image depth, Image confidence, CameraIntrinsics cameraTextureIntrinsics, float[] modelMatrix) {
        // Java uses big endian so we have to change the endianess to ensure we extract
        // depth data in the correct byte order.
        final Image.Plane depthImagePlane = depth.getPlanes()[0];
        ByteBuffer depthByteBufferOriginal = depthImagePlane.getBuffer();
        ByteBuffer depthByteBuffer = ByteBuffer.allocate(depthByteBufferOriginal.capacity());
        depthByteBuffer.order(ByteOrder.LITTLE_ENDIAN);
        while (depthByteBufferOriginal.hasRemaining()) {
            depthByteBuffer.put(depthByteBufferOriginal.get());
        }
        depthByteBuffer.rewind();
        ShortBuffer depthBuffer = depthByteBuffer.asShortBuffer();

        final Image.Plane confidenceImagePlane = confidence.getPlanes()[0];
        ByteBuffer confidenceBufferOriginal = confidenceImagePlane.getBuffer();
        ByteBuffer confidenceBuffer = ByteBuffer.allocate(confidenceBufferOriginal.capacity());
        confidenceBuffer.order(ByteOrder.LITTLE_ENDIAN);
        while (confidenceBufferOriginal.hasRemaining()) {
            confidenceBuffer.put(confidenceBufferOriginal.get());
        }
        confidenceBuffer.rewind();

        // To transform 2D depth pixels into 3D points we retrieve the intrinsic camera parameters
        // corresponding to the depth image. See more information about the depth values at
        // https://developers.google.com/ar/develop/java/depth/overview#understand-depth-values.
        final int[] intrinsicsDimensions = cameraTextureIntrinsics.getImageDimensions();
        final int depthWidth = depth.getWidth();
        final int depthHeight = depth.getHeight();
        final float fx =
                cameraTextureIntrinsics.getFocalLength()[0] * depthWidth / intrinsicsDimensions[0];
        final float fy =
                cameraTextureIntrinsics.getFocalLength()[1] * depthHeight / intrinsicsDimensions[1];
        final float cx =
                cameraTextureIntrinsics.getPrincipalPoint()[0] * depthWidth / intrinsicsDimensions[0];
        final float cy =
                cameraTextureIntrinsics.getPrincipalPoint()[1] * depthHeight / intrinsicsDimensions[1];

        // Allocate the destination point buffer. If the number of depth pixels is larger than
        // `maxNumberOfPointsToRender` we uniformly subsample. The raw depth image may have
        // different resolutions on different devices.
        final float maxNumberOfPointsToRender = 20000;
        int step = (int) Math.ceil(Math.sqrt(depthWidth * depthHeight / maxNumberOfPointsToRender));
        System.out.println("step: "+Integer.toString(step));
        FloatBuffer points = FloatBuffer.allocate(depthWidth / step * depthHeight / step * FLOATS_PER_POINT);
        float[] pointCamera = new float[4];
        float[] pointWorld = new float[4];
        float[] RGBWorld = new float[3];

        Bitmap cameraImageBitmap=convertYUVtoBitmap(cameraImage,depthWidth,depthHeight);

        for (int y = 0; y < depthHeight; y += step) {
            for (int x = 0; x < depthWidth; x += step) {
                // Depth images are tightly packed, so it's OK to not use row and pixel strides.
                int depthMillimeters = depthBuffer.get(y * depthWidth + x); // Depth image pixels are in mm.

                if (depthMillimeters == 0) {
                    // Pixels with value zero are invalid, meaning depth estimates are missing from
                    // this location.
                    continue;
                }
                final float depthMeters = depthMillimeters / 1000.0f; // Depth image pixels are in mm.

                // Retrieves the confidence value for this pixel.
                final byte confidencePixelValue =
                        confidenceBuffer.get(
                                y * confidenceImagePlane.getRowStride()
                                        + x * confidenceImagePlane.getPixelStride());
                final float confidenceNormalized = ((float) (confidencePixelValue & 0xff)) / 255.0f;
                if (confidenceNormalized < 0.2 || depthMeters > 1.5) {
                    // Ignores "low-confidence" pixels.
                    continue;
                }

                // Unprojects the depth into a 3D point in camera coordinates.
                pointCamera[0] = depthMeters * (x - cx) / fx;
                pointCamera[1] = depthMeters * (cy - y) / fy;
                pointCamera[2] = -depthMeters;
                pointCamera[3] = 1;


                // Applies model matrix to transform point into world coordinates.
                Matrix.multiplyMV(pointWorld, 0, modelMatrix, 0, pointCamera, 0);

                RGBWorld=getRGBValueFromBitmap(cameraImageBitmap,x,y);

                points.put(pointWorld[0]); // X.
                points.put(pointWorld[1]); // Y.
                points.put(pointWorld[2]); // Z.
                points.put(RGBWorld[0]);    //R
                points.put(RGBWorld[1]);    //G
                points.put(RGBWorld[2]);    //B
//                points.put(confidenceNormalized);
            }
        }

        points.rewind();
        return points;
    }
    public static float[] getRGBValueFromBitmap(Bitmap bitmap,int x,int y){
        int colour = bitmap.getPixel(x, y);

        float red = (float)Color.red(colour);
        float green = (float)Color.green(colour);
        float blue = (float)Color.blue(colour);
        float alpha = (float)Color.alpha(colour);
        return new float[]{red,green,blue};
    }
    public static Bitmap convertYUVtoBitmap(Image cameraImage,int scaledWidth,int scaledHeight){
        byte[] nv21;
        // Get the three planes.
        ByteBuffer yBuffer = cameraImage.getPlanes()[0].getBuffer();
        ByteBuffer uBuffer = cameraImage.getPlanes()[1].getBuffer();
        ByteBuffer vBuffer =cameraImage.getPlanes()[2].getBuffer();

        int ySize = yBuffer.remaining();
        int uSize = uBuffer.remaining();
        int vSize = vBuffer.remaining();


        nv21 = new byte[ySize + uSize + vSize];

        //U and V are swapped
        yBuffer.get(nv21, 0, ySize);
        vBuffer.get(nv21, ySize, vSize);
        uBuffer.get(nv21, ySize + vSize, uSize);


        int width = cameraImage.getWidth();
        int height = cameraImage.getHeight();

        ByteArrayOutputStream out = new ByteArrayOutputStream();
        YuvImage yuv = new YuvImage(nv21, ImageFormat.NV21, width, height, null);
        yuv.compressToJpeg(new Rect(0, 0, width, height), 100, out);
        byte[] byteArray = out.toByteArray();
        Bitmap bitmap = BitmapFactory.decodeByteArray(byteArray, 0, byteArray.length);
        bitmap = Bitmap.createScaledBitmap(bitmap, scaledWidth, scaledHeight, true);
        return bitmap;
    }
    /** Obtain the depth in millimeters for depthImage at coordinates (x, y). */
    public static float getCentimetersDepth(Image depthImage, int x, int y) {
        // The depth image has a single plane, which stores depth for each
        // pixel as 16-bit unsigned integers.
//        Image.Plane plane = depthImage.getPlanes()[0];
//        int byteIndex = x * plane.getPixelStride() + y * plane.getRowStride();
//        ByteBuffer buffer = plane.getBuffer().order(ByteOrder.nativeOrder());
//        float depthValue=(float)buffer.getShort(byteIndex)/10.0f;
        final Image.Plane depthImagePlane = depthImage.getPlanes()[0];
        ByteBuffer depthByteBufferOriginal = depthImagePlane.getBuffer();
        ByteBuffer depthByteBuffer = ByteBuffer.allocate(depthByteBufferOriginal.capacity());
        depthByteBuffer.order(ByteOrder.LITTLE_ENDIAN);
        while (depthByteBufferOriginal.hasRemaining()) {
            depthByteBuffer.put(depthByteBufferOriginal.get());
        }
        depthByteBuffer.rewind();
        ShortBuffer depthBuffer = depthByteBuffer.asShortBuffer();
        final int depthWidth = depthImage.getWidth();
        int depthMillimeters = depthBuffer.get(y * depthWidth + x);
        float depthValue=depthMillimeters/10.0f;
        return depthValue;
    }
    public static float getConfidenceDepth(Image confidenceImage, int x, int y) {
        // The depth image has a single plane, which stores depth for each
        // pixel as 16-bit unsigned integers.
        final Image.Plane confidenceImagePlane = confidenceImage.getPlanes()[0];
        ByteBuffer confidenceBufferOriginal = confidenceImagePlane.getBuffer();
        ByteBuffer confidenceBuffer = ByteBuffer.allocate(confidenceBufferOriginal.capacity());
        confidenceBuffer.order(ByteOrder.LITTLE_ENDIAN);
        while (confidenceBufferOriginal.hasRemaining()) {
            confidenceBuffer.put(confidenceBufferOriginal.get());
        }
        confidenceBuffer.rewind();
        final byte confidencePixelValue =
                confidenceBuffer.get(
                        y * confidenceImagePlane.getRowStride()
                                + x * confidenceImagePlane.getPixelStride());
        final float confidenceNormalized = ((float) (confidencePixelValue & 0xff)) / 255.0f;
        return confidenceNormalized;
    }
    public static byte[] convertFloatBufferToByteArray(FloatBuffer fB){
        fB.rewind();
        System.out.println("fB.remaining() "+fB.remaining());
        System.out.println("fB.capacity() "+fB.capacity());
        ByteBuffer byteBuffer = ByteBuffer.allocate(fB.capacity() * 4);
        byteBuffer.order(fB.order());
        byteBuffer.asFloatBuffer().put(fB);
        System.out.println("converted byteBuffer byte_order "+byteBuffer.order());
        byte[] bytearray = byteBuffer.array();
        return bytearray;
    }

    public static void filterUsingPlanes(FloatBuffer points, Collection<Plane> allPlanes) {
        float[] planeNormal = new float[3];

        // Allocates the output buffer.
        int numPoints = points.remaining() / DepthData.FLOATS_PER_POINT;

        // Each plane is checked against each point.
        for (Plane plane : allPlanes) {
            if (plane.getTrackingState() != TrackingState.TRACKING || plane.getSubsumedBy() != null) {
                continue;
            }

            // Computes the normal vector of the plane.
            Pose planePose = plane.getCenterPose();
            planePose.getTransformedAxis(1, 1.0f, planeNormal, 0);

            // Filters points that are too close to the plane.
            for (int index = 0; index < numPoints; ++index) {
                // Retrieves the next point.
                final float x = points.get(FLOATS_PER_POINT * index);
                final float y = points.get(FLOATS_PER_POINT * index + 1);
                final float z = points.get(FLOATS_PER_POINT * index + 2);

                // Transforms point to be in world coordinates, to match plane info.
                float distance = (x - planePose.tx()) * planeNormal[0]
                        + (y - planePose.ty()) * planeNormal[1]
                        + (z - planePose.tz()) * planeNormal[2];

                // Controls the size of objects detected.
                // Smaller values mean smaller objects will be kept.
                // Larger values will only allow detection of larger objects, but also helps reduce noise.
                if (Math.abs(distance) > 0.03) {
                    continue;  // Keeps this point, since it's far enough away from the plane.
                }

                // Invalidates points that are too close to planar surfaces.
                points.put(FLOATS_PER_POINT * index, 0);
                points.put(FLOATS_PER_POINT * index + 1, 0);
                points.put(FLOATS_PER_POINT * index + 2, 0);
                points.put(FLOATS_PER_POINT * index + 3, 0);
            }
        }
    }
}
