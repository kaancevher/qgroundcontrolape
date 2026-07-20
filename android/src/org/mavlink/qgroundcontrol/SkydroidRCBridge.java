package org.mavlink.qgroundcontrol;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Toast;

import com.skydroid.rcsdk.PipelineManager;
import com.skydroid.rcsdk.RCSDKManager;
import com.skydroid.rcsdk.SDKManagerCallBack;
import com.skydroid.rcsdk.comm.CommListener;
import com.skydroid.rcsdk.common.Uart;
import com.skydroid.rcsdk.common.error.SkyException;
import com.skydroid.rcsdk.common.pipeline.Pipeline;

/**
 * Direct bidirectional bridge between Skydroid H12 RCSDK UART0
 * and QGroundControl's native H12Link.
 */
public final class SkydroidRCBridge {
    private static final String TAG = "QGC-RCSDK";

    private static final Object LOCK = new Object();
    private static final Object PIPELINE_OPERATION_LOCK = new Object();

    private static final Handler MAIN_HANDLER =
            new Handler(Looper.getMainLooper());

    private static Context applicationContext = null;
    private static SDKManagerCallBack sdkCallback = null;
    private static CommListener pipelineListener = null;
    private static Pipeline pipeline = null;

    private static long generationCounter = 0;
    private static long activeGeneration = 0;

    private static boolean lifecycleStarted = false;
    private static boolean acceptingCallbacks = false;
    private static boolean stopping = false;
    private static boolean nativeLinkAttached = false;
    private static boolean rcConnected = false;
    private static boolean pipelineCreationAttempted = false;
    private static boolean pipelineConnectInvoked = false;
    private static boolean pipelineConnected = false;

    private static boolean rcConnectedToastShown = false;
    private static boolean pipelineConnectedToastShown = false;
    private static boolean firstTelemetryToastShown = false;
    private static boolean fatalStartupToastShown = false;

    private SkydroidRCBridge() {
    }

    /*
     * Registered by H12Link::setNativeMethods() during JNI_OnLoad.
     */
    private static native void nativeOnData(byte[] data);
    private static native void nativeOnPipelineState(boolean connected);
    private static native void nativeOnError(String message);

    public static void start(Context context) {
        if (context == null) {
            Log.e(TAG, "Cannot start RCSDK bridge with null context");
            return;
        }

        Context appContext = context.getApplicationContext();

        if (appContext == null) {
            appContext = context;
        }

        final long generation;
        final SDKManagerCallBack callback;

        synchronized (LOCK) {
            if (lifecycleStarted || stopping) {
                Log.i(TAG, "RCSDK bridge lifecycle already started");
                return;
            }

            lifecycleStarted = true;
            acceptingCallbacks = true;
            applicationContext = appContext;
            activeGeneration = ++generationCounter;
            generation = activeGeneration;

            nativeLinkAttached = false;
            rcConnected = false;
            pipelineCreationAttempted = false;
            pipelineConnectInvoked = false;
            pipelineConnected = false;

            rcConnectedToastShown = false;
            pipelineConnectedToastShown = false;
            firstTelemetryToastShown = false;
            fatalStartupToastShown = false;

            callback = createSdkCallback(generation);
            sdkCallback = callback;
        }

        showStatus("RCSDK hazırlanıyor");

        if (Looper.myLooper() != Looper.getMainLooper()) {
            fatalStartup(
                    generation,
                    "RCSDK bridge must start on the Android main thread",
                    null
            );
            return;
        }

        try {
            RCSDKManager.INSTANCE.initSDK(appContext, callback);
            RCSDKManager.INSTANCE.setMainThreadCallBack(true);
            RCSDKManager.INSTANCE.connectToRC();
            Log.i(TAG, "RCSDK initialization and RC connection requested");
        } catch (Throwable error) {
            fatalStartup(
                    generation,
                    "RCSDK startup failed",
                    error
            );
        }
    }

    public static void stop() {
        final Pipeline pipelineToDisconnect;
        final boolean shouldDisconnectRc;

        synchronized (LOCK) {
            if (!lifecycleStarted || stopping) {
                return;
            }

            stopping = true;
            acceptingCallbacks = false;
            nativeLinkAttached = false;
            activeGeneration = 0;

            pipelineToDisconnect = pipeline;
            pipeline = null;
            pipelineListener = null;
            sdkCallback = null;

            rcConnected = false;
            pipelineConnected = false;
            shouldDisconnectRc = true;
        }

        synchronized (PIPELINE_OPERATION_LOCK) {
            if (pipelineToDisconnect != null) {
                try {
                    PipelineManager.INSTANCE.disconnectPipeline(
                            pipelineToDisconnect
                    );
                } catch (Throwable error) {
                    Log.w(TAG, "RCSDK disconnectPipeline failed", error);
                }

                try {
                    pipelineToDisconnect.setOnCommListener(null);
                } catch (Throwable error) {
                    Log.w(TAG, "Could not clear UART0 listener", error);
                }
            }

            if (shouldDisconnectRc) {
                try {
                    RCSDKManager.INSTANCE.disconnectRC();
                } catch (Throwable error) {
                    Log.w(TAG, "RCSDK disconnectRC failed", error);
                }
            }
        }

        synchronized (LOCK) {
            applicationContext = null;
            lifecycleStarted = false;
            stopping = false;
        }

        Log.i(TAG, "RCSDK bridge stopped");
    }

    /**
     * Called from H12Link::_connect(). This only attaches JNI delivery;
     * it does not initialize or reconnect RCSDK.
     */
    public static boolean attachNativeLink() {
        final boolean accepted;

        synchronized (LOCK) {
            accepted =
                    lifecycleStarted
                            && acceptingCallbacks
                            && !stopping;

            if (accepted) {
                nativeLinkAttached = true;
            }
        }

        if (accepted) {
            notifyNativePipelineState(currentPipelineState());
        }

        return accepted;
    }

    /**
     * Called from H12Link teardown. RCSDK and UART0 remain owned by the
     * application lifecycle.
     */
    public static void detachNativeLink() {
        synchronized (LOCK) {
            nativeLinkAttached = false;
        }
    }

    /**
     * Called from H12Link::_writeBytes().
     */
    public static boolean writeData(byte[] data) {
        if (data == null || data.length == 0) {
            return false;
        }

        synchronized (PIPELINE_OPERATION_LOCK) {
            final Pipeline currentPipeline;

            synchronized (LOCK) {
                if (!lifecycleStarted
                        || !acceptingCallbacks
                        || stopping
                        || !nativeLinkAttached
                        || pipeline == null) {
                    return false;
                }

                currentPipeline = pipeline;
            }

            try {
                currentPipeline.writeData(data);
                return true;
            } catch (Throwable error) {
                reportError(
                        "RCSDK UART0 writeData failed",
                        error
                );
                return false;
            }
        }
    }

    private static SDKManagerCallBack createSdkCallback(
            final long generation) {
        return new SDKManagerCallBack() {
            @Override
            public void onRcConnected() {
                boolean showConnected = false;

                synchronized (LOCK) {
                    if (!isLifecycleActiveLocked(generation)) {
                        return;
                    }

                    rcConnected = true;

                    if (!rcConnectedToastShown) {
                        rcConnectedToastShown = true;
                        showConnected = true;
                    }
                }

                Log.i(TAG, "H12 RC connected");

                if (showConnected) {
                    showStatus("H12 kumanda bağlantısı kuruldu");
                }

                createAndConnectPipelineOnce(generation);
            }

            @Override
            public void onRcConnectFail(SkyException error) {
                synchronized (LOCK) {
                    if (!isLifecycleActiveLocked(generation)) {
                        return;
                    }

                    rcConnected = false;
                    pipelineConnected = false;
                }

                reportError(
                        "H12 RC connection failed: "
                                + String.valueOf(error),
                        error
                );
                notifyNativePipelineState(false);
            }

            @Override
            public void onRcDisconnect() {
                synchronized (LOCK) {
                    if (!isLifecycleActiveLocked(generation)) {
                        return;
                    }

                    rcConnected = false;
                    pipelineConnected = false;
                }

                Log.w(TAG, "H12 RC disconnected; RCSDK recovery remains active");
                notifyNativePipelineState(false);
            }
        };
    }

    private static void createAndConnectPipelineOnce(
            final long generation) {
        synchronized (LOCK) {
            if (!isLifecycleActiveLocked(generation)
                    || pipelineCreationAttempted) {
                return;
            }

            pipelineCreationAttempted = true;
        }

        final Pipeline newPipeline;

        try {
            newPipeline =
                    PipelineManager.INSTANCE.createPipeline(
                            Uart.UART0
                    );

            if (newPipeline == null) {
                throw new IllegalStateException(
                        "RCSDK returned a null UART0 pipeline"
                );
            }
        } catch (Throwable error) {
            fatalStartup(
                    generation,
                    "Could not create RCSDK UART0 pipeline",
                    error
            );
            return;
        }

        final CommListener listener =
                createPipelineListener(generation, newPipeline);

        try {
            newPipeline.setOnCommListener(listener);
        } catch (Throwable error) {
            fatalStartup(
                    generation,
                    "Could not install RCSDK UART0 listener",
                    error
            );
            return;
        }

        synchronized (LOCK) {
            if (!isLifecycleActiveLocked(generation)
                    || pipeline != null) {
                try {
                    newPipeline.setOnCommListener(null);
                } catch (Throwable error) {
                    Log.w(TAG, "Could not clear stale UART0 listener", error);
                }
                return;
            }

            // Store before connectPipeline so synchronous callbacks can
            // validate pipeline identity.
            pipeline = newPipeline;
            pipelineListener = listener;
        }

        synchronized (PIPELINE_OPERATION_LOCK) {
            synchronized (LOCK) {
                if (!isCurrentPipelineLocked(
                        generation,
                        newPipeline)
                        || pipelineConnectInvoked) {
                    return;
                }

                pipelineConnectInvoked = true;
            }

            try {
                PipelineManager.INSTANCE.connectPipeline(newPipeline);
                Log.i(TAG, "RCSDK UART0 pipeline connection requested once");
            } catch (Throwable error) {
                fatalStartup(
                        generation,
                        "RCSDK connectPipeline failed",
                        error
                );
            }
        }
    }

    private static CommListener createPipelineListener(
            final long generation,
            final Pipeline ownerPipeline) {
        return new CommListener() {
            @Override
            public void onConnectSuccess() {
                boolean showConnected = false;

                synchronized (LOCK) {
                    if (!isCurrentPipelineLocked(
                            generation,
                            ownerPipeline)) {
                        return;
                    }

                    pipelineConnected = true;

                    if (!pipelineConnectedToastShown) {
                        pipelineConnectedToastShown = true;
                        showConnected = true;
                    }
                }

                Log.i(TAG, "RCSDK UART0 pipeline connected");
                notifyNativePipelineState(true);

                if (showConnected) {
                    showStatus("H12 UART0 telemetri hattı hazır");
                }
            }

            @Override
            public void onConnectFail(SkyException error) {
                synchronized (LOCK) {
                    if (!isCurrentPipelineLocked(
                            generation,
                            ownerPipeline)) {
                        return;
                    }

                    pipelineConnected = false;
                }

                reportError(
                        "RCSDK UART0 connection failed: "
                                + String.valueOf(error),
                        error
                );
                notifyNativePipelineState(false);
            }

            @Override
            public void onDisconnect() {
                synchronized (LOCK) {
                    if (!isCurrentPipelineLocked(
                            generation,
                            ownerPipeline)) {
                        return;
                    }

                    pipelineConnected = false;
                }

                Log.w(
                        TAG,
                        "RCSDK UART0 pipeline disconnected; "
                                + "internal recovery remains active"
                );
                notifyNativePipelineState(false);
            }

            @Override
            public void onReadData(byte[] data) {
                if (data == null || data.length == 0) {
                    return;
                }

                boolean showFirstTelemetry = false;

                synchronized (LOCK) {
                    if (!isCurrentPipelineLocked(
                            generation,
                            ownerPipeline)
                            || !nativeLinkAttached) {
                        return;
                    }

                    try {
                        // Forward the original RCSDK byte array directly.
                        nativeOnData(data);
                    } catch (Throwable error) {
                        Log.e(TAG, "nativeOnData callback failed", error);
                        return;
                    }

                    if (!firstTelemetryToastShown) {
                        firstTelemetryToastShown = true;
                        showFirstTelemetry = true;
                    }
                }

                if (showFirstTelemetry) {
                    showStatus("H12 telemetri verisi alındı");
                }
            }
        };
    }

    private static boolean isLifecycleActiveLocked(long generation) {
        return lifecycleStarted
                && acceptingCallbacks
                && !stopping
                && activeGeneration == generation;
    }

    private static boolean isCurrentPipelineLocked(
            long generation,
            Pipeline ownerPipeline) {
        return isLifecycleActiveLocked(generation)
                && pipeline == ownerPipeline;
    }

    private static boolean currentPipelineState() {
        synchronized (LOCK) {
            return pipelineConnected;
        }
    }

    private static void notifyNativePipelineState(boolean connected) {
        synchronized (LOCK) {
            if (!lifecycleStarted
                    || !acceptingCallbacks
                    || stopping
                    || !nativeLinkAttached) {
                return;
            }

            try {
                nativeOnPipelineState(connected);
            } catch (Throwable error) {
                Log.e(
                        TAG,
                        "nativeOnPipelineState callback failed",
                        error
                );
            }
        }
    }

    private static void reportError(
            String message,
            Throwable error) {
        if (error == null) {
            Log.e(TAG, message);
        } else {
            Log.e(TAG, message, error);
        }

        synchronized (LOCK) {
            if (!lifecycleStarted
                    || !acceptingCallbacks
                    || stopping
                    || !nativeLinkAttached) {
                return;
            }

            try {
                nativeOnError(message);
            } catch (Throwable callbackError) {
                Log.e(
                        TAG,
                        "nativeOnError callback failed",
                        callbackError
                );
            }
        }
    }

    private static void fatalStartup(
            long generation,
            String message,
            Throwable error) {
        boolean showFatal = false;

        synchronized (LOCK) {
            if (!lifecycleStarted
                    || stopping
                    || activeGeneration != generation) {
                return;
            }

            acceptingCallbacks = false;
            nativeLinkAttached = false;
            rcConnected = false;
            pipelineConnected = false;

            if (!fatalStartupToastShown) {
                fatalStartupToastShown = true;
                showFatal = true;
            }
        }

        if (error == null) {
            Log.e(TAG, message);
        } else {
            Log.e(TAG, message, error);
        }

        if (showFatal) {
            showStatus("H12 RCSDK başlatılamadı");
        }
    }

    private static void showStatus(final String message) {
        Log.i(TAG, message);

        final Context context;

        synchronized (LOCK) {
            context = applicationContext;
        }

        if (context == null) {
            return;
        }

        MAIN_HANDLER.post(new Runnable() {
            @Override
            public void run() {
                try {
                    Toast.makeText(
                            context,
                            message,
                            Toast.LENGTH_LONG
                    ).show();
                } catch (Throwable error) {
                    Log.w(TAG, "Could not show RCSDK status", error);
                }
            }
        });
    }
}
