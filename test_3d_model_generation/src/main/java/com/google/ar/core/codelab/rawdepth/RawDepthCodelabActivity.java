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

import android.content.ContentResolver;
import android.content.ContentValues;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.Image;
import android.net.Uri;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.widget.ImageView;
import android.widget.Toast;
import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Camera;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Plane;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import com.google.ar.core.codelab.common.helpers.AABB;
import com.google.ar.core.codelab.common.helpers.CameraPermissionHelper;
import com.google.ar.core.codelab.common.helpers.DisplayRotationHelper;
import com.google.ar.core.codelab.common.helpers.FullScreenHelper;
import com.google.ar.core.codelab.common.helpers.PointClusteringHelper;
import com.google.ar.core.codelab.common.helpers.SnackbarHelper;
import com.google.ar.core.codelab.common.helpers.TrackingStateHelper;
import com.google.ar.core.codelab.common.rendering.BackgroundRenderer;
import com.google.ar.core.codelab.common.rendering.BoxRenderer;
import com.google.ar.core.codelab.common.rendering.DepthRenderer;
import com.google.ar.core.codelab.rawdepth.ml.Model;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.core.exceptions.UnavailableApkTooOldException;
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;
import com.google.ar.core.exceptions.UnavailableSdkTooOldException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import android.widget.Button;
import android.widget.TextView;

import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;


import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Objects;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * This is a simple example that shows how to create an augmented reality (AR) application using the
 * ARCore Raw Depth API. The application will show 3D point-cloud data of the environment.
 */
public class RawDepthCodelabActivity extends AppCompatActivity implements GLSurfaceView.Renderer {
  private static final String TAG = RawDepthCodelabActivity.class.getSimpleName();

  // Rendering. The Renderers are created here, and initialized when the GL surface is created.
  private GLSurfaceView surfaceView;

  private boolean installRequested;
  private boolean showDepthMap = false;
  private boolean takePicture = false;

  private Session session;
  private final SnackbarHelper messageSnackbarHelper = new SnackbarHelper();
  private DisplayRotationHelper displayRotationHelper;

  private final DepthTextureHandler depthTexture = new DepthTextureHandler();
  private final DepthRenderer depthRenderer = new DepthRenderer();
  private final BackgroundRenderer backgroundRenderer = new BackgroundRenderer();
  private boolean isDepthSupported;
  private ImageView tv1;
  private int saveState=1;
  private static int imageCount=0;
  private static int maxNumImage=5;
//  private final BoxRenderer boxRenderer = new BoxRenderer();

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    surfaceView = findViewById(R.id.surfaceview);
    displayRotationHelper = new DisplayRotationHelper(/*context=*/ this);

    // Set up renderer.
    surfaceView.setPreserveEGLContextOnPause(true);
    surfaceView.setEGLContextClientVersion(2);
    surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0); // Alpha used for plane blending.
    surfaceView.setRenderer(this);
    surfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    surfaceView.setWillNotDraw(false);

    installRequested = false;

    final Button toggleDepthButton = (Button) findViewById(R.id.toggle_depth_button);
    toggleDepthButton.setOnClickListener(
            view -> {
                showDepthMap = !showDepthMap;
                toggleDepthButton.setText(showDepthMap ? R.string.hide_depth : R.string.show_depth);
            });

    final Button takePictureButton = (Button) findViewById(R.id.take_picture);
    takePictureButton.setOnClickListener(
            view -> {
              takePicture=true;
            });
    tv1=(ImageView) findViewById(R.id.imageView);

  }
  public void takeImageCamera(Frame frame){
    takePicture=false;
    try {
      Image cameraImage = frame.acquireCameraImage();

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
        //resize to 512x512
      bitmap = Bitmap.createScaledBitmap(bitmap, 512, 512, true);

      String timeStamp = new SimpleDateFormat("ddMMyyyy_HHmm").format(new Date());
      String mImageName="MI_"+ timeStamp;
      saveImage(bitmap,mImageName);

      System.out.println("cameraImageWidth: "+Integer.toString(width)+" cameraImageHeight: "+Integer.toString(height));
      tv1.setImageBitmap(bitmap);

      cameraImage.close();



    }catch (NotYetAvailableException | IOException e ) {
      // This normally means that depth data is not available yet. This is normal so we will not
      // spam the logcat with this.
    }
  }
  private void saveImage(Bitmap bitmap, @NonNull String name) throws IOException {
    OutputStream fos;
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
      ContentResolver resolver = getContentResolver();
      ContentValues contentValues = new ContentValues();
      contentValues.put(MediaStore.MediaColumns.DISPLAY_NAME, name + ".jpg");
      contentValues.put(MediaStore.MediaColumns.MIME_TYPE, "image/jpg");
      contentValues.put(MediaStore.MediaColumns.RELATIVE_PATH, Environment.DIRECTORY_PICTURES);
      Uri imageUri = resolver.insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, contentValues);
      fos = resolver.openOutputStream(Objects.requireNonNull(imageUri));
    } else {
      String imagesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES).toString();
      File image = new File(imagesDir, name + ".jpg");
      fos = new FileOutputStream(image);
    }
    bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fos);
    Objects.requireNonNull(fos).close();
  }
  private void createAndSaveFile() {
      if(saveState==1){
          Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT);

          intent.addCategory(Intent.CATEGORY_OPENABLE);
          intent.setType("text/plain");
          intent.putExtra(Intent.EXTRA_TITLE, "rawDepthPointCloudData.txt");
          startActivityForResult(intent, 1);
      }else if(saveState==2){
          Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT);

          intent.addCategory(Intent.CATEGORY_OPENABLE);
          intent.setType("text/plain");
          intent.putExtra(Intent.EXTRA_TITLE, "rawDepthData.txt");
          startActivityForResult(intent, 2);
      }





  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
    super.onActivityResult(requestCode, resultCode, data);
    takePicture=false;
    if (requestCode == 1) {

      if (resultCode == RESULT_OK) {
        try {
          Uri uri = data.getData();

          OutputStream outputStream = getContentResolver().openOutputStream(uri);

          outputStream.write(DepthData.rawDepthPointCloudData);

          outputStream.close();

          Toast.makeText(this, "Write rawDepthPointCloudData successfully", Toast.LENGTH_SHORT).show();
        } catch (IOException e) {
          Toast.makeText(this, "Fail to write  rawDepthPointCloudData", Toast.LENGTH_SHORT).show();
        }
      } else {
        Toast.makeText(this, " rawDepthPointCloudData not saved", Toast.LENGTH_SHORT).show();
      }
      saveState=2;
      createAndSaveFile();

    }else if(requestCode==2){
        if (resultCode == RESULT_OK) {
            try {
                Uri uri = data.getData();

                OutputStream outputStream = getContentResolver().openOutputStream(uri);

                outputStream.write(DepthData.rawDepthData);

                outputStream.close();

                Toast.makeText(this, "Write rawDepthData successfully", Toast.LENGTH_SHORT).show();
            } catch (IOException e) {
                Toast.makeText(this, "Fail to write rawDepthData", Toast.LENGTH_SHORT).show();
            }
        } else {
            Toast.makeText(this, "rawDepthData not saved", Toast.LENGTH_SHORT).show();
        }
        saveState=1;
    }
  }


  @Override
  protected void onResume() {
    super.onResume();

    if (session == null) {
      Exception exception = null;
      String message = null;
      try {
        switch (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
          case INSTALL_REQUESTED:
            installRequested = true;
            return;
          case INSTALLED:
            break;
        }

        // ARCore requires camera permissions to operate. If we did not yet obtain runtime
        // permission on Android M and above, now is a good time to ask the user for it.
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
          CameraPermissionHelper.requestCameraPermission(this);
          return;
        }

        // Creates the ARCore session.
        session = new Session(/* context= */ this);
        if (!session.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY)) {
          message =
                  "This device does not support the ARCore Raw Depth API. See"
                          + " https://developers.google.com/ar/discover/supported-devices.";
        }

      } catch (UnavailableArcoreNotInstalledException
          | UnavailableUserDeclinedInstallationException e) {
        message = "Please install ARCore";
        exception = e;
      } catch (UnavailableApkTooOldException e) {
        message = "Please update ARCore";
        exception = e;
      } catch (UnavailableSdkTooOldException e) {
        message = "Please update this app";
        exception = e;
      } catch (UnavailableDeviceNotCompatibleException e) {
        message = "This device does not support AR";
        exception = e;
      } catch (Exception e) {
        message = "Failed to create AR session";
        exception = e;
      }

      if (message != null) {
        messageSnackbarHelper.showError(this, message);
        Log.e(TAG, "Exception creating session", exception);
        return;
      }
    }

    try {
      // Enable raw depth estimation and auto focus mode while ARCore is running.
      Config config = session.getConfig();
      config.setDepthMode(Config.DepthMode.RAW_DEPTH_ONLY);
      config.setFocusMode(Config.FocusMode.AUTO);
      session.configure(config);
      session.resume();
    } catch (CameraNotAvailableException e) {
      messageSnackbarHelper.showError(this, "Camera not available. Try restarting the app.");
      session = null;
      return;
    }

    // Note that order matters - see the note in onPause(), the reverse applies here.
    surfaceView.onResume();
    displayRotationHelper.onResume();
    messageSnackbarHelper.showMessage(this, "Waiting for depth data...");
  }

  @Override
  public void onPause() {
    super.onPause();
    if (session != null) {
      // Note that the order matters - GLSurfaceView is paused first so that it does not try
      // to query the session. If Session is paused before GLSurfaceView, GLSurfaceView may
      // still call session.update() and get a SessionPausedException.
      displayRotationHelper.onPause();
      surfaceView.onPause();
      session.pause();
    }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
    if (!CameraPermissionHelper.hasCameraPermission(this)) {
      Toast.makeText(this, "Camera permission is needed to run this application",
          Toast.LENGTH_LONG).show();
      if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
        // Permission denied with checking "Do not ask again".
        CameraPermissionHelper.launchPermissionSettings(this);
      }
      finish();
    }
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus);
  }

  @Override
  public void onSurfaceCreated(GL10 gl, EGLConfig config) {

    GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // Prepare the rendering objects. This involves reading shaders, so may throw an IOException.
    try {
      // Create the texture and pass it to ARCore session to be filled during update().
      depthTexture.createOnGlThread();
      backgroundRenderer.createOnGlThread(/*context=*/ this);
      depthRenderer.createOnGlThread(/*context=*/ this);
      // Add to onSurfaceCreated() after backgroundRenderer.createonGlThread(/*context=*/ this);
      backgroundRenderer.createDepthShaders(/*context=*/ this, depthTexture.getDepthTexture());
      //boxRenderer.createOnGlThread(/*context=*/this);
    } catch (IOException e) {
      Log.e(TAG, "Failed to read an asset file", e);
    }
  }

  @Override
  public void onSurfaceChanged(GL10 gl, int width, int height) {

    displayRotationHelper.onSurfaceChanged(width, height);
    GLES20.glViewport(0, 0, width, height);
  }

  @Override
  public void onDrawFrame(GL10 gl) {
    // Clear screen to notify driver it should not load any pixels from previous frame.
    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

    if (session == null) {
      return;
    }
    // Notify ARCore session that the view size changed so that the perspective matrix and
    // the video background can be properly adjusted.
    displayRotationHelper.updateSessionIfNeeded(session);

    try {
      session.setCameraTextureName(backgroundRenderer.getTextureId());

      // Obtain the current frame from ARSession. When the configuration is set to
      // UpdateMode.BLOCKING (it is by default), this will throttle the rendering to the
      // camera framerate.
      Frame frame = session.update();
      Camera camera = frame.getCamera();

      depthTexture.update(frame);
      // If frame is ready, render camera preview image to the GL surface.
      backgroundRenderer.draw(frame);
      // Add this snippet just under backgroundRenderer.draw(frame);
      if (showDepthMap) {
        backgroundRenderer.drawDepth(frame);
      }

//      // Retrieve the depth data for this frame.
//      FloatBuffer points = DepthData.create(frame, session.createAnchor(camera.getPose()));
//      if (points == null) {
//        return;
//      }
//
//      if (messageSnackbarHelper.isShowing() && points != null) {
//        messageSnackbarHelper.hide(this);
//      }
//
//      // If not tracking, show tracking failure reason instead.
//      if (camera.getTrackingState() == TrackingState.PAUSED) {
//        messageSnackbarHelper.showMessage(
//            this, TrackingStateHelper.getTrackingFailureReasonString(camera));
//        return;
//      }

      // Filters the depth data.
//     DepthData.filterUsingPlanes(points, session.getAllTrackables(Plane.class));

      // Visualize depth points.
//      if (!showDepthMap) {
//        depthRenderer.update(points);
//        depthRenderer.draw(camera);
//      }
      // Draw boxes around clusters of points.
//      PointClusteringHelper clusteringHelper = new PointClusteringHelper(points);
//      List<AABB> clusters = clusteringHelper.findClusters();
//      for (AABB aabb : clusters) {
//        boxRenderer.draw(aabb, camera);
//      }
      if(takePicture){
        takeImageCamera(frame);
        takePicture=false;
          // Retrieve the depth data for this frame.
          FloatBuffer points = DepthData.create(frame, session.createAnchor(camera.getPose()));
          if (points == null) {
              return;
          }
        imageCount+=1;
        System.out.println("imageCount "+Integer.toString(imageCount));
        if(imageCount==maxNumImage){
          createAndSaveFile();
          imageCount=0;
        }

          if (messageSnackbarHelper.isShowing() && points != null) {
              messageSnackbarHelper.hide(this);
          }

          // If not tracking, show tracking failure reason instead.
          if (camera.getTrackingState() == TrackingState.PAUSED) {
              messageSnackbarHelper.showMessage(
                      this, TrackingStateHelper.getTrackingFailureReasonString(camera));
              return;
          }


      }

    } catch (Throwable t) {
      // Avoid crashing the application due to unhandled exceptions.
      Log.e(TAG, "Exception on the OpenGL thread", t);
    }
  }
}
