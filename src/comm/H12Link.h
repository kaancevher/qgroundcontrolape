/****************************************************************************
 *
 * Skydroid H12 RCSDK native MAVLink link for QGroundControl.
 *
 ****************************************************************************/

#pragma once

#ifdef __android__

#include "LinkInterface.h"

#include <QLoggingCategory>

#include <atomic>

Q_DECLARE_LOGGING_CATEGORY(H12LinkLog)

class H12Link final : public LinkInterface
{
    Q_OBJECT

public:
    explicit H12Link(SharedLinkConfigurationPtr& config);
    ~H12Link() override;

    bool isConnected(void) const override;
    void disconnect(void) override;

    /// Registers Java -> C++ callbacks during JNI_OnLoad.
    static bool setNativeMethods(void);

    /// Internal SerialConfiguration marker used by LinkManager.
    static QString configurationPortName(void);

private slots:
    void _writeBytes(const QByteArray data) override;

    /// Called on the H12Link QObject thread after JNI receives UART0 bytes.
    void _handleIncomingData(const QByteArray data);

    /// Diagnostic RCSDK UART0 state; does not own the logical QGC link.
    void _handlePipelineState(bool connected);

    /// Receives a diagnostic message from the Java bridge.
    void _handleBridgeError(const QString message);

private:
    bool _connect(void) override;

    std::atomic_bool _logicalConnected  { false };
    std::atomic_bool _pipelineAvailable { false };
};

#endif // __android__
