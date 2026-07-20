/****************************************************************************
 *
 * Skydroid H12 RCSDK native MAVLink link for QGroundControl.
 *
 ****************************************************************************/

#include "H12Link.h"

#ifdef __android__

#include "QGCLoggingCategory.h"

#include <QtAndroidExtras/QAndroidJniEnvironment>
#include <QtAndroidExtras/QAndroidJniObject>

#include <QMetaObject>
#include <QMutex>
#include <QMutexLocker>

#include <jni.h>

QGC_LOGGING_CATEGORY(H12LinkLog, "H12LinkLog")

namespace {

static const char kBridgeClassName[] =
        "org/mavlink/qgroundcontrol/SkydroidRCBridge";

QMutex   s_activeLinkMutex;
H12Link* s_activeLink = nullptr;

void clearActiveLink(H12Link* link)
{
    QMutexLocker locker(&s_activeLinkMutex);

    if (s_activeLink == link) {
        s_activeLink = nullptr;
    }
}

bool clearPendingJniException(
        QAndroidJniEnvironment& env,
        const char* operation)
{
    if (!env->ExceptionCheck()) {
        return false;
    }

    qCWarning(H12LinkLog)
            << "JNI exception during"
            << operation;

    env->ExceptionDescribe();
    env->ExceptionClear();
    return true;
}

void nativeOnData(
        JNIEnv* env,
        jclass,
        jbyteArray dataArray)
{
    if (!env || !dataArray) {
        return;
    }

    const jsize length = env->GetArrayLength(dataArray);

    if (length <= 0) {
        return;
    }

    QByteArray data(static_cast<int>(length), Qt::Uninitialized);

    env->GetByteArrayRegion(
            dataArray,
            0,
            length,
            reinterpret_cast<jbyte*>(data.data()));

    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        return;
    }

    QMutexLocker locker(&s_activeLinkMutex);

    if (!s_activeLink) {
        return;
    }

    if (!QMetaObject::invokeMethod(
            s_activeLink,
            "_handleIncomingData",
            Qt::QueuedConnection,
            Q_ARG(QByteArray, data))) {
        qCWarning(H12LinkLog)
                << "Could not queue incoming RCSDK UART0 data";
    }
}

void nativeOnPipelineState(
        JNIEnv*,
        jclass,
        jboolean connected)
{
    QMutexLocker locker(&s_activeLinkMutex);

    if (!s_activeLink) {
        return;
    }

    if (!QMetaObject::invokeMethod(
            s_activeLink,
            "_handlePipelineState",
            Qt::QueuedConnection,
            Q_ARG(bool, connected == JNI_TRUE))) {
        qCWarning(H12LinkLog)
                << "Could not queue RCSDK pipeline state";
    }
}

void nativeOnError(
        JNIEnv*,
        jclass,
        jstring message)
{
    QString errorMessage =
            QStringLiteral("Unknown RCSDK error");

    if (message) {
        errorMessage =
                QAndroidJniObject(message).toString();
    }

    QMutexLocker locker(&s_activeLinkMutex);

    if (!s_activeLink) {
        return;
    }

    if (!QMetaObject::invokeMethod(
            s_activeLink,
            "_handleBridgeError",
            Qt::QueuedConnection,
            Q_ARG(QString, errorMessage))) {
        qCWarning(H12LinkLog)
                << "Could not queue RCSDK bridge error";
    }
}

} // namespace

H12Link::H12Link(SharedLinkConfigurationPtr& config)
    : LinkInterface(config)
{
    qCDebug(H12LinkLog)
            << "H12Link created:"
            << config->name();
}

H12Link::~H12Link()
{
    disconnect();

    // Also clears a partially registered _connect() attempt.
    clearActiveLink(this);
}

QString H12Link::configurationPortName(void)
{
    return QStringLiteral("skydroid-h12-rcsdk-uart0");
}

bool H12Link::setNativeMethods(void)
{
    JNINativeMethod methods[] = {
        {
            const_cast<char*>("nativeOnData"),
            const_cast<char*>("([B)V"),
            reinterpret_cast<void*>(nativeOnData)
        },
        {
            const_cast<char*>("nativeOnPipelineState"),
            const_cast<char*>("(Z)V"),
            reinterpret_cast<void*>(nativeOnPipelineState)
        },
        {
            const_cast<char*>("nativeOnError"),
            const_cast<char*>("(Ljava/lang/String;)V"),
            reinterpret_cast<void*>(nativeOnError)
        }
    };

    QAndroidJniEnvironment env;

    jclass bridgeClass =
            env->FindClass(kBridgeClassName);

    if (!bridgeClass) {
        qCWarning(H12LinkLog)
                << "Could not find Java bridge class:"
                << kBridgeClassName;

        clearPendingJniException(
                env,
                "FindClass SkydroidRCBridge");

        return false;
    }

    const jint result =
            env->RegisterNatives(
                    bridgeClass,
                    methods,
                    sizeof(methods) / sizeof(methods[0]));

    const bool hadException =
            clearPendingJniException(
                    env,
                    "RegisterNatives SkydroidRCBridge");

    env->DeleteLocalRef(bridgeClass);

    if (result != JNI_OK || hadException) {
        qCWarning(H12LinkLog)
                << "RegisterNatives failed:"
                << result;
        return false;
    }

    qCDebug(H12LinkLog)
            << "Skydroid H12 native callbacks registered";

    return true;
}

bool H12Link::_connect(void)
{
    if (_logicalConnected.load()) {
        return true;
    }

    _pipelineAvailable.store(false);

    {
        QMutexLocker locker(&s_activeLinkMutex);

        if (s_activeLink && s_activeLink != this) {
            qCWarning(H12LinkLog)
                    << "Another H12Link is already active";
            return false;
        }

        s_activeLink = this;
    }

    QAndroidJniEnvironment env;

    const jboolean attached =
            QAndroidJniObject::callStaticMethod<jboolean>(
                    kBridgeClassName,
                    "attachNativeLink",
                    "()Z");

    if (clearPendingJniException(
            env,
            "SkydroidRCBridge.attachNativeLink")
            || attached != JNI_TRUE) {
        qCWarning(H12LinkLog)
                << "Java RCSDK bridge rejected native attachment";

        clearActiveLink(this);
        _pipelineAvailable.store(false);
        return false;
    }

    const bool wasConnected =
            _logicalConnected.exchange(true);

    if (!wasConnected) {
        qCDebug(H12LinkLog)
                << "H12 native link attached";
        emit connected();
    }

    return true;
}

bool H12Link::isConnected(void) const
{
    return _logicalConnected.load();
}

void H12Link::disconnect(void)
{
    const bool wasConnected =
            _logicalConnected.exchange(false);

    _pipelineAvailable.store(false);

    // Prevent new JNI callbacks from being queued before Java detach
    // or object destruction can continue.
    clearActiveLink(this);

    if (!wasConnected) {
        return;
    }

    QAndroidJniEnvironment env;

    QAndroidJniObject::callStaticMethod<void>(
            kBridgeClassName,
            "detachNativeLink",
            "()V");

    clearPendingJniException(
            env,
            "SkydroidRCBridge.detachNativeLink");

    qCDebug(H12LinkLog)
            << "H12 native link disconnected";

    emit disconnected();
}

void H12Link::_writeBytes(const QByteArray data)
{
    if (!_logicalConnected.load() || data.isEmpty()) {
        return;
    }

    QAndroidJniEnvironment env;

    jbyteArray javaData =
            env->NewByteArray(data.size());

    if (!javaData) {
        clearPendingJniException(
                env,
                "NewByteArray");
        return;
    }

    env->SetByteArrayRegion(
            javaData,
            0,
            data.size(),
            reinterpret_cast<const jbyte*>(data.constData()));

    if (clearPendingJniException(
            env,
            "SetByteArrayRegion")) {
        env->DeleteLocalRef(javaData);
        return;
    }

    const jboolean written =
            QAndroidJniObject::callStaticMethod<jboolean>(
                    kBridgeClassName,
                    "writeData",
                    "([B)Z",
                    javaData);

    const bool hadException =
            clearPendingJniException(
                    env,
                    "SkydroidRCBridge.writeData");

    env->DeleteLocalRef(javaData);

    if (!hadException && written == JNI_TRUE) {
        emit bytesSent(this, data);
    } else {
        qCDebug(H12LinkLog)
                << "RCSDK UART0 did not accept outgoing data";
    }
}

void H12Link::_handleIncomingData(const QByteArray data)
{
    if (!_logicalConnected.load() || data.isEmpty()) {
        return;
    }

    emit bytesReceived(this, data);
}

void H12Link::_handlePipelineState(bool connected)
{
    if (!_logicalConnected.load()) {
        return;
    }

    const bool previous =
            _pipelineAvailable.exchange(connected);

    if (previous == connected) {
        return;
    }

    if (connected) {
        qCDebug(H12LinkLog)
                << "RCSDK UART0 pipeline available";
    } else {
        qCWarning(H12LinkLog)
                << "RCSDK UART0 pipeline unavailable; "
                   "logical H12 link remains attached";
    }
}

void H12Link::_handleBridgeError(const QString message)
{
    if (!_logicalConnected.load()) {
        return;
    }

    qCWarning(H12LinkLog)
            << "RCSDK bridge:"
            << message;
}

#endif // __android__
