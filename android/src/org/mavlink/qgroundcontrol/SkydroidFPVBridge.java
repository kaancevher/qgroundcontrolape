package org.mavlink.qgroundcontrol;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.Gravity;
import android.view.ViewGroup;
import android.view.ViewParent;
import android.widget.FrameLayout;
import android.widget.Toast;

import com.skydroid.fpvplayer.FPVReaderWidget;
import com.skydroid.rcsdk.PipelineManager;
import com.skydroid.rcsdk.comm.CommListener;
import com.skydroid.rcsdk.common.error.SkyException;
import com.skydroid.rcsdk.common.pipeline.Pipeline;

/**
 * Process-wide smoke-test bridge for the Skydroid H12 serial video stream.
 */
public final class SkydroidFPVBridge {
    private static final String TAG = "QGC-H12Video";

    private static final int VIDEO_WIDTH = 640;
    private static final int VIDEO_HEIGHT = 360;
    private static final int VIDEO_FRAME_RATE = 15;
    private static final int OVERLAY_MARGIN = 16;

    private static final Object LOCK = new Object();
    private static final Handler MAIN_HANDLER = new Handler(Looper.getMainLooper());

    private static QGCActivity activity = null;
    private static FPVReaderWidget fpvReaderWidget = null;
    private static Pipeline videoPipeline = null;
    private static CommListener videoListener = null;

    private static boolean lifecycleStarted = false;
    private static boolean acceptingCallbacks = false;
    private static boolean stopping = false;
    private static boolean pipelineConnected = false;

    private static long generationCounter = 0;
    private static long activeGeneration = 0;

    private static boolean startingToastShown = false;
    private static boolean connectedToastShown = false;
    private static boolean firstDataToastShown = false;
    private static boolean fatalStartupToastShown = false;

    private SkydroidFPVBridge() {
    }

    public static void start(final QGCActivity qgcActivity) {
        if (qgcActivity == null) {
            Log.e(TAG, "Cannot start H12 video bridge with null activity");
            return;
        }

        if (Looper.myLooper() != Looper.getMainLooper()) {
            MAIN_HANDLER.post(new Runnable() {
                @Override
                public void run() {
                    start(qgcActivity);
                }
            });
            return;
        }

        final long generation;

        synchronized (LOCK) {
            if (lifecycleStarted || stopping) {
                Log.i(TAG, "H12 video bridge lifecycle already started");
                return;
            }

            lifecycleStarted = true;
            acceptingCallbacks = true;
            activity = qgcActivity;
            pipelineConnected = false;
            activeGeneration = ++generationCounter;
            generation = activeGeneration;

            startingToastShown = false;
            connectedToastShown = false;
            firstDataToastShown = false;
            fatalStartupToastShown = false;
        }

        showStartingToast();

        try {
            FPVReaderWidget newWidget = new FPVReaderWidget(qgcActivity);
            synchronized (LOCK) {
                fpvReaderWidget = newWidget;
            }

            newWidget.setClickable(false);
            newWidget.setLongClickable(false);
            newWidget.setFocusable(false);
            newWidget.setVideoDecoderType(
                    FPVReaderWidget.VIDEO_DECODER_TYPE_OPENH264
            );

            FrameLayout.LayoutParams layoutParams = new FrameLayout.LayoutParams(
                    VIDEO_WIDTH,
                    VIDEO_HEIGHT,
                    Gravity.BOTTOM | Gravity.END
            );
            layoutParams.setMargins(0, 0, OVERLAY_MARGIN, OVERLAY_MARGIN);
            qgcActivity.addContentView(newWidget, layoutParams);

            Pipeline newPipeline =
                    PipelineManager.INSTANCE.createSerialPipeline("/dev/ttyHS0", 4000000);
            CommListener newListener = createVideoListener(generation, newPipeline);
            synchronized (LOCK) {
                videoPipeline = newPipeline;
                videoListener = newListener;
            }
            newPipeline.setOnCommListener(newListener);

            PipelineManager.INSTANCE.connectPipeline(newPipeline);
            newWidget.start(
                    FPVReaderWidget.VIDEO_TYPE_H264,
                    VIDEO_WIDTH,
                    VIDEO_HEIGHT,
                    VIDEO_FRAME_RATE
            );
            Log.i(TAG, "H12 serial video startup requested");
        } catch (Throwable error) {
            fatalStartup(error);
        }
    }

    public static void stop() {
        if (Looper.myLooper() != Looper.getMainLooper()) {
            MAIN_HANDLER.post(new Runnable() {
                @Override
                public void run() {
                    stop();
                }
            });
            return;
        }

        releaseResources(true);
    }

    private static CommListener createVideoListener(
            final long expectedGeneration,
            final Pipeline expectedPipeline
    ) {
        return new CommListener() {
            @Override
            public void onConnectSuccess() {
                boolean showConnectedToast = false;

                synchronized (LOCK) {
                    if (!isExpectedLifecycleLocked(
                            expectedGeneration,
                            expectedPipeline,
                            this
                    )) {
                        return;
                    }

                    pipelineConnected = true;
                    if (!connectedToastShown) {
                        connectedToastShown = true;
                        showConnectedToast = true;
                    }
                }

                Log.i(TAG, "H12 video serial pipeline connected");
                if (showConnectedToast) {
                    showActiveToast("H12 video seri hattı bağlandı");
                }
            }

            @Override
            public void onConnectFail(SkyException error) {
                synchronized (LOCK) {
                    if (!isExpectedLifecycleLocked(
                            expectedGeneration,
                            expectedPipeline,
                            this
                    )) {
                        return;
                    }
                    pipelineConnected = false;
                }

                Log.w(TAG, "H12 video serial pipeline connection failed", error);
            }

            @Override
            public void onDisconnect() {
                synchronized (LOCK) {
                    if (!isExpectedLifecycleLocked(
                            expectedGeneration,
                            expectedPipeline,
                            this
                    )) {
                        return;
                    }
                    pipelineConnected = false;
                }

                Log.w(TAG, "H12 video serial pipeline disconnected");
            }

            @Override
            public void onReadData(byte[] data) {
                if (data == null || data.length == 0) {
                    return;
                }

                final FPVReaderWidget widget;
                boolean showFirstDataToast = false;

                synchronized (LOCK) {
                    if (!isExpectedLifecycleLocked(
                            expectedGeneration,
                            expectedPipeline,
                            this
                    ) || fpvReaderWidget == null) {
                        return;
                    }
                    widget = fpvReaderWidget;
                }

                try {
                    widget.sendFrame(data);
                } catch (Throwable error) {
                    Log.e(TAG, "Could not forward H12 video data", error);
                    return;
                }

                synchronized (LOCK) {
                    if (!firstDataToastShown) {
                        firstDataToastShown = true;
                        showFirstDataToast = true;
                    }
                }

                if (showFirstDataToast) {
                    Log.i(TAG, "First H12 video data received");
                    showActiveToast("H12 ilk video verisi alındı");
                }
            }
        };
    }

    private static boolean isExpectedLifecycleLocked(
            long expectedGeneration,
            Pipeline expectedPipeline,
            CommListener expectedListener
    ) {
        return lifecycleStarted
                && acceptingCallbacks
                && !stopping
                && activeGeneration == expectedGeneration
                && videoPipeline == expectedPipeline
                && videoListener == expectedListener;
    }

    private static void showStartingToast() {
        synchronized (LOCK) {
            if (startingToastShown || activity == null) {
                return;
            }
            startingToastShown = true;
            Toast.makeText(
                    activity,
                    "H12 video başlatılıyor",
                    Toast.LENGTH_SHORT
            ).show();
        }
    }

    private static void showActiveToast(final String message) {
        MAIN_HANDLER.post(new Runnable() {
            @Override
            public void run() {
                synchronized (LOCK) {
                    if (!acceptingCallbacks || activity == null) {
                        return;
                    }
                    Toast.makeText(activity, message, Toast.LENGTH_SHORT).show();
                }
            }
        });
    }

    private static void fatalStartup(Throwable error) {
        Log.e(TAG, "H12 video startup failed", error);

        synchronized (LOCK) {
            if (!fatalStartupToastShown && activity != null) {
                fatalStartupToastShown = true;
                Toast.makeText(
                        activity,
                        "H12 video başlatma hatası",
                        Toast.LENGTH_LONG
                ).show();
            }
        }

        releaseResources(true);
    }

    private static void releaseResources(boolean finishLifecycle) {
        final FPVReaderWidget widgetToStop;
        final Pipeline pipelineToDisconnect;

        synchronized (LOCK) {
            if (!lifecycleStarted || stopping) {
                return;
            }

            stopping = true;
            acceptingCallbacks = false;
            pipelineConnected = false;
            activeGeneration = ++generationCounter;

            widgetToStop = fpvReaderWidget;
            pipelineToDisconnect = videoPipeline;

            fpvReaderWidget = null;
            videoPipeline = null;
            videoListener = null;
            activity = null;
        }

        if (widgetToStop != null) {
            try {
                widgetToStop.stop();
            } catch (Throwable error) {
                Log.w(TAG, "Could not stop H12 video widget", error);
            }
        }

        if (pipelineToDisconnect != null) {
            try {
                pipelineToDisconnect.setOnCommListener(null);
            } catch (Throwable error) {
                Log.w(TAG, "Could not clear H12 video serial listener", error);
            }

            try {
                PipelineManager.INSTANCE.disconnectPipeline(pipelineToDisconnect);
            } catch (Throwable error) {
                Log.w(TAG, "Could not disconnect H12 video serial pipeline", error);
            }
        }

        if (widgetToStop != null) {
            ViewParent parent = widgetToStop.getParent();
            if (parent instanceof ViewGroup) {
                ((ViewGroup) parent).removeView(widgetToStop);
            }
        }

        synchronized (LOCK) {
            if (finishLifecycle) {
                lifecycleStarted = false;
            }
            stopping = false;
        }
    }
}
