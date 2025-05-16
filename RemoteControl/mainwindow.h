#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <QDebug>
#include <QtWidgets>
#include <QBluetoothPermission>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFrame>
#include <QFont>
#include <QDoubleValidator>
#include <QMessageBox>
#include "message.h"
#include "bluetoothclient.h"

#if defined(Q_OS_ANDROID)
#include <QJniObject>
#include <QJniEnvironment>
#endif

namespace Ui {
class MainWindow;
}

class TouchFriendlyParameterWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TouchFriendlyParameterWidget(const QString &labelText,
                                          float minValue,
                                          float maxValue,
                                          int decimals,
                                          QWidget *parent = nullptr);

    void setValue(float value);
    float getValue() const;

signals:
    void valueChanged(float value);

private slots:
    void increaseValue();
    void decreaseValue();
    void onValueEdited();

private:
    QLabel *m_label;
    QLineEdit *m_valueEdit;
    QPushButton *m_plusButton;
    QPushButton *m_minusButton;

    float m_minValue;
    float m_maxValue;
    float m_currentValue;
    int m_decimals;
    float m_step;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // Bluetooth and data handling
    void DataHandler(QByteArray data);
    void changedState(BluetoothClient::bluetoothleState state);
    void statusChanged(const QString &status);

    // Command sending methods
    void sendCommand(uint8_t command, uint8_t value);
    void sendFloatCommand(uint8_t command, float value);
    void sendString(uint8_t command, QByteArray value);
    void requestData(uint8_t command);

    // Button handlers
    void onConnectClicked();
    void onForwardPressed();
    void onForwardReleased();
    void onBackwardPressed();
    void onBackwardReleased();
    void onRightPressed();
    void onRightReleased();
    void onLeftPressed();
    void onLeftReleased();
    void onExitClicked();
    void onSpeakClicked();
    void onTestClicked();
    void onArmedClicked();

    // Parameter value changed handlers
    void onPValueChanged(float value);
    void onDValueChanged(float value);
    void onCValueChanged(float value);
    void onVValueChanged(float value);
    void onACValueChanged(float value);

private:
    // Initialization methods
    void createUI();
    void setupConnections();
    void setupTouchFriendlyUI();
    void setupBluetoothConnection();

    // Message handling methods
    void createMessage(uint8_t msgId, uint8_t rw, QByteArray payload, QByteArray *result);
    void parseMessage(QByteArray *data, uint8_t &command, QByteArray &value, uint8_t &rw);

// Platform-specific methods
#if defined(Q_OS_ANDROID)
    void requestBluetoothPermissions();
#endif

#if defined(Q_OS_IOS)
    void requestiOSBluetoothPermissions();
    bool m_iOSBluetoothInitialized = false;
#endif

    // UI components
    QWidget *m_centralWidget;
    QVBoxLayout *m_mainLayout;

    // Status area
    QLabel *m_textStatus;

    // Control buttons
    QPushButton *m_connectButton;
    QPushButton *m_armedButton;
    QPushButton *m_exitButton;

    // Direction buttons
    QPushButton *m_forwardButton;
    QPushButton *m_backwardButton;
    QPushButton *m_leftButton;
    QPushButton *m_rightButton;

    // Parameter widgets
    TouchFriendlyParameterWidget *m_pParameter;
    TouchFriendlyParameterWidget *m_dParameter;
    TouchFriendlyParameterWidget *m_cParameter;
    TouchFriendlyParameterWidget *m_vParameter;
    TouchFriendlyParameterWidget *m_acParameter;

    // Voice command widgets
    QLineEdit *m_speakEdit;
    QPushButton *m_speakButton;
    QPushButton *m_testButton;

    // Parameter value definitions
    const float KP_MIN = 0.0f;
    const float KP_MAX = 0.5f;
    const float KD_MIN = 0.0f;
    const float KD_MAX = 10.0f;
    const float KC_MIN = 0.0f;
    const float KC_MAX = 0.001f;
    const float KV_MIN = 0.0f;
    const float KV_MAX = 0.05f;
    const float AC_MIN = 0.0f;
    const float AC_MAX = 5.0f;

    // Member variables
    int remoteConstant;
    float m_scaleFactor;
    BluetoothClient *m_bleConnection;
    Message message;
};

#endif // MAINWINDOW_H
