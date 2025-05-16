#include "mainwindow.h"

TouchFriendlyParameterWidget::TouchFriendlyParameterWidget(const QString &labelText, float minValue, float maxValue, int decimals, QWidget *parent)
    : QWidget(parent)
    , m_minValue(minValue)
    , m_maxValue(maxValue)
    , m_currentValue(0.0f)
    , m_decimals(decimals)
{
    float range = maxValue - minValue;
    m_step = range / 100.0f;

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(10);

    m_label = new QLabel(labelText, this);
    m_label->setStyleSheet("color: white; background-color: #FF4633; padding: 8px; border-radius: 5px;");
    QFont labelFont;
    labelFont.setPointSize(16);
    labelFont.setBold(true);
    m_label->setFont(labelFont);
    m_label->setMinimumWidth(60);
    m_label->setMaximumHeight(60);

    m_valueEdit = new QLineEdit(this);
    m_valueEdit->setStyleSheet("color: white; background-color: #336699; padding: 12px; border-radius: 5px; font-size: 16px;");
    m_valueEdit->setAlignment(Qt::AlignCenter);
    m_valueEdit->setMinimumWidth(75);
    m_valueEdit->setMaximumHeight(60);

    QDoubleValidator *validator = new QDoubleValidator(minValue, maxValue, decimals, this);
    validator->setNotation(QDoubleValidator::StandardNotation);
    m_valueEdit->setValidator(validator);

    m_minusButton = new QPushButton("-", this);
    m_plusButton = new QPushButton("+", this);

    QString buttonStyle = "QPushButton { color: white; background-color: #336699; padding: 12px; border-radius: 5px; font-size: 24px; font-weight: bold; min-width: 60px; } "
                          "QPushButton:pressed { background-color: #1E3F5A; }";
    m_minusButton->setStyleSheet(buttonStyle);
    m_minusButton->setMaximumHeight(60);
    m_plusButton->setStyleSheet(buttonStyle);
    m_plusButton->setMaximumHeight(60);

    layout->addWidget(m_label);
    layout->addWidget(m_minusButton);
    layout->addWidget(m_valueEdit);
    layout->addWidget(m_plusButton);

    connect(m_plusButton, &QPushButton::clicked, this, &TouchFriendlyParameterWidget::increaseValue);
    connect(m_minusButton, &QPushButton::clicked, this, &TouchFriendlyParameterWidget::decreaseValue);
    connect(m_valueEdit, &QLineEdit::editingFinished, this, &TouchFriendlyParameterWidget::onValueEdited);

    setValue(minValue);
}

void TouchFriendlyParameterWidget::setValue(float value)
{
    value = qBound(m_minValue, value, m_maxValue);

    if (m_currentValue != value) {
        m_currentValue = value;
        m_valueEdit->setText(QString::number(value, 'f', m_decimals));
        emit valueChanged(value);
    }
}

float TouchFriendlyParameterWidget::getValue() const
{
    return m_currentValue;
}

void TouchFriendlyParameterWidget::increaseValue()
{
    setValue(m_currentValue + m_step);
}

void TouchFriendlyParameterWidget::decreaseValue()
{
    setValue(m_currentValue - m_step);
}

void TouchFriendlyParameterWidget::onValueEdited()
{
    bool ok;
    float value = m_valueEdit->text().toFloat(&ok);

    if (ok) {
        setValue(value);
    } else {
        m_valueEdit->setText(QString::number(m_currentValue, 'f', m_decimals));
    }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , remoteConstant(50)
    , m_scaleFactor(1.0f)
    , m_bleConnection(nullptr)
{
    setWindowTitle(tr("BalanceRobot Remote Control"));

#ifdef Q_OS_ANDROID
    m_scaleFactor = 1.25f;
#endif

    createUI();
    setupBluetoothConnection();
    setupConnections();

    statusChanged("No device connected.");
}

MainWindow::~MainWindow()
{
    if (m_bleConnection) {
        m_bleConnection->disconnectFromDevice();
        delete m_bleConnection;
    }
}

void MainWindow::createUI()
{
    m_centralWidget = new QWidget(this);
    setCentralWidget(m_centralWidget);

    m_mainLayout = new QVBoxLayout(m_centralWidget);
    m_mainLayout->setSpacing(15);
    m_mainLayout->setContentsMargins(10, 10, 10, 10);

    m_textStatus = new QLabel(this);
    m_textStatus->setStyleSheet("color: #cccccc; background-color: #003333; border-radius: 5px; padding: 10px;");
    QFont statusFont;
    statusFont.setPointSize(static_cast<int>(12 * m_scaleFactor));
    m_textStatus->setFont(statusFont);
    m_textStatus->setMaximumHeight(60);

    QHBoxLayout *connectionLayout = new QHBoxLayout();
    connectionLayout->setSpacing(10);

    m_connectButton = new QPushButton("Connect", this);
    m_armedButton = new QPushButton("Arm", this);
    m_exitButton = new QPushButton("Exit", this);

    QString buttonStyle = "QPushButton { color: white; background-color: #336699; padding: 10px; "
                          "border-radius: 8px; font-size: 18px; font-weight: bold; min-height: 25px; } "
                          "QPushButton:pressed { background-color: #1E3F5A; }";

    m_connectButton->setStyleSheet(buttonStyle);
    m_armedButton->setStyleSheet(buttonStyle);
    m_exitButton->setStyleSheet(buttonStyle);

    connectionLayout->addWidget(m_connectButton);
    connectionLayout->addWidget(m_armedButton);
    connectionLayout->addWidget(m_exitButton);

    m_mainLayout->addLayout(connectionLayout);

    QGridLayout *directionLayout = new QGridLayout();
    directionLayout->setSpacing(5);

    m_forwardButton = new QPushButton(this);
    m_backwardButton = new QPushButton(this);
    m_leftButton = new QPushButton(this);
    m_rightButton = new QPushButton(this);

    int iconSize = static_cast<int>(50 * m_scaleFactor);
    QSize buttonSize(iconSize * 1.25, iconSize);

    QPixmap forwardPixmap(":/icons/forward.png");
    QPixmap backwardPixmap(":/icons/back.png");
    QPixmap leftPixmap(":/icons/left.png");
    QPixmap rightPixmap(":/icons/right.png");

    m_forwardButton->setIcon(QIcon(forwardPixmap.scaled(buttonSize, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    m_backwardButton->setIcon(QIcon(backwardPixmap.scaled(buttonSize, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    m_leftButton->setIcon(QIcon(leftPixmap.scaled(buttonSize, Qt::KeepAspectRatio, Qt::SmoothTransformation)));
    m_rightButton->setIcon(QIcon(rightPixmap.scaled(buttonSize, Qt::KeepAspectRatio, Qt::SmoothTransformation)));

    QString dirButtonStyle = "QPushButton { background-color: rgba(51, 102, 153, 180); border-radius: 10px; padding: 10px; }"
                             "QPushButton:pressed { background-color: rgba(30, 63, 90, 200); }";

    m_forwardButton->setStyleSheet(dirButtonStyle);
    m_backwardButton->setStyleSheet(dirButtonStyle);
    m_leftButton->setStyleSheet(dirButtonStyle);
    m_rightButton->setStyleSheet(dirButtonStyle);

    m_forwardButton->setIconSize(buttonSize);
    m_backwardButton->setIconSize(buttonSize);
    m_leftButton->setIconSize(buttonSize);
    m_rightButton->setIconSize(buttonSize);

    m_forwardButton->setFixedSize(buttonSize);
    m_backwardButton->setFixedSize(buttonSize);
    m_leftButton->setFixedSize(buttonSize);
    m_rightButton->setFixedSize(buttonSize);

    directionLayout->addWidget(m_forwardButton, 0, 1, Qt::AlignCenter);
    directionLayout->addWidget(m_leftButton, 1, 0, Qt::AlignCenter);
    directionLayout->addWidget(m_rightButton, 1, 2, Qt::AlignCenter);
    directionLayout->addWidget(m_backwardButton, 2, 1, Qt::AlignCenter);

    QFrame *directionFrame = new QFrame(this);
    directionFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
    directionFrame->setStyleSheet("background-color: rgba(208, 211, 212, 255); border-radius: 15px;");
    directionFrame->setLayout(directionLayout);

    QGroupBox *parameterGroup = new QGroupBox("Control Parameters", this);
    parameterGroup->setStyleSheet("QGroupBox { color: black; font-size: 18px; font-weight: bold; "
                                  "border: 2px solid #336699; border-radius: 10px; margin-top: 15px; padding: 10px; } "
                                  "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 10px; }");

    QVBoxLayout *parameterLayout = new QVBoxLayout(parameterGroup);
    parameterLayout->setSpacing(10);

    m_pParameter = new TouchFriendlyParameterWidget("P", KP_MIN, KP_MAX, 3, this);
    m_dParameter = new TouchFriendlyParameterWidget("D", KD_MIN, KD_MAX, 3, this);
    m_cParameter = new TouchFriendlyParameterWidget("C", KC_MIN, KC_MAX, 4, this);
    m_vParameter = new TouchFriendlyParameterWidget("V", KV_MIN, KV_MAX, 2, this);
    m_acParameter = new TouchFriendlyParameterWidget("AC", AC_MIN, AC_MAX, 2, this);

    m_pParameter->setValue(0);
    m_dParameter->setValue(0);
    m_cParameter->setValue(0);
    m_vParameter->setValue(0);
    m_acParameter->setValue(0);

    parameterLayout->addWidget(m_pParameter);
    parameterLayout->addWidget(m_dParameter);
    parameterLayout->addWidget(m_cParameter);
    parameterLayout->addWidget(m_vParameter);
    parameterLayout->addWidget(m_acParameter);

    QHBoxLayout *speakLayout = new QHBoxLayout();

    m_speakEdit = new QLineEdit(this);
    m_speakEdit->setStyleSheet("color: white; background-color: #FF4633; padding: 12px; border-radius: 5px; font-size: 16px;");
    m_speakEdit->setText("Merhaba. Nasılsın?");
    m_speakEdit->setPlaceholderText("Enter speech command...");

    m_speakButton = new QPushButton("Speak", this);
    m_testButton = new QPushButton("Test Motors", this);

    m_speakButton->setStyleSheet("QPushButton { color: white; background-color: #900C3F; padding: 12px; "
                                 "border-radius: 5px; font-size: 16px; font-weight: bold; } "
                                 "QPushButton:pressed { background-color: #5A0627; }");

    m_testButton->setStyleSheet("QPushButton { color: white; background-color: #336699; padding: 12px; "
                                  "border-radius: 5px; font-size: 16px; font-weight: bold; } "
                                  "QPushButton:pressed { background-color: #1E3F5A; }");

    speakLayout->addWidget(m_speakEdit, 3);
    speakLayout->addWidget(m_speakButton, 1);
    speakLayout->addWidget(m_testButton, 1);

    m_mainLayout->addWidget(m_textStatus);
    m_mainLayout->addWidget(parameterGroup);
    m_mainLayout->addWidget(directionFrame);
    m_mainLayout->addLayout(speakLayout);
}

void MainWindow::setupConnections()
{
    connect(m_connectButton, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
    connect(m_exitButton, &QPushButton::clicked, this, &MainWindow::onExitClicked);
    connect(m_armedButton, &QPushButton::clicked, this, &MainWindow::onArmedClicked);

    connect(m_forwardButton, &QPushButton::pressed, this, &MainWindow::onForwardPressed);
    connect(m_forwardButton, &QPushButton::released, this, &MainWindow::onForwardReleased);
    connect(m_backwardButton, &QPushButton::pressed, this, &MainWindow::onBackwardPressed);
    connect(m_backwardButton, &QPushButton::released, this, &MainWindow::onBackwardReleased);
    connect(m_leftButton, &QPushButton::pressed, this, &MainWindow::onLeftPressed);
    connect(m_leftButton, &QPushButton::released, this, &MainWindow::onLeftReleased);
    connect(m_rightButton, &QPushButton::pressed, this, &MainWindow::onRightPressed);
    connect(m_rightButton, &QPushButton::released, this, &MainWindow::onRightReleased);

    connect(m_speakButton, &QPushButton::clicked, this, &MainWindow::onSpeakClicked);
    connect(m_testButton, &QPushButton::clicked, this, &MainWindow::onTestClicked);

    connect(m_pParameter, &TouchFriendlyParameterWidget::valueChanged, this, &MainWindow::onPValueChanged);
    connect(m_dParameter, &TouchFriendlyParameterWidget::valueChanged, this, &MainWindow::onDValueChanged);
    connect(m_cParameter, &TouchFriendlyParameterWidget::valueChanged, this, &MainWindow::onCValueChanged);
    connect(m_vParameter, &TouchFriendlyParameterWidget::valueChanged, this, &MainWindow::onVValueChanged);
    connect(m_acParameter, &TouchFriendlyParameterWidget::valueChanged, this, &MainWindow::onACValueChanged);
}

void MainWindow::setupBluetoothConnection()
{
    m_bleConnection = new BluetoothClient();

    connect(m_bleConnection, &BluetoothClient::statusChanged, this, &MainWindow::statusChanged);
    connect(m_bleConnection, &BluetoothClient::changedState, this, &MainWindow::changedState);
}

void MainWindow::statusChanged(const QString &status)
{
    m_textStatus->setText(status);
    update();
}

void MainWindow::changedState(BluetoothClient::bluetoothleState state)
{
    switch(state) {
    case BluetoothClient::Scanning:
        statusChanged("Searching for low energy devices...");
        break;

    case BluetoothClient::ScanFinished:
        statusChanged("Scan finished...");
        break;

    case BluetoothClient::Connecting:
        statusChanged("Connecting to device...");
        break;

    case BluetoothClient::Connected:
        m_connectButton->setText("Disconnect");
        statusChanged("Connected successfully.");
        connect(m_bleConnection, &BluetoothClient::newData, this, &MainWindow::DataHandler);
        break;

    case BluetoothClient::DisConnected:
        statusChanged("Device disconnected.");
        m_connectButton->setEnabled(true);
        m_connectButton->setText("Connect");

        m_pParameter->setValue(0);
        m_dParameter->setValue(0);
        m_cParameter->setValue(0);
        m_vParameter->setValue(0);
        m_acParameter->setValue(0);
        break;

    case BluetoothClient::ServiceFound:
        break;

    case BluetoothClient::AcquireData:
        requestData(mPP);
        requestData(mPD);
        requestData(mPC);
        requestData(mPV);
        requestData(mAC);
        requestData(mArmed);
        break;

    case BluetoothClient::Error:
        statusChanged("ERROR: Bluetooth operation failed.");
        break;

    default:
        break;
    }
}

void MainWindow::DataHandler(QByteArray data)
{
    uint8_t parsedCommand;
    uint8_t rw;
    QByteArray parsedValue;
    parseMessage(&data, parsedCommand, parsedValue, rw);

    if(rw == mWrite) {
        switch(parsedCommand) {
        case mPP:
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));
            m_pParameter->setValue(rawValue);
            qDebug() << "KP (Proportional):" << rawValue;
            break;
        }
        case mPD:
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));
            m_dParameter->setValue(rawValue);
            qDebug() << "KD (Derivative):" << rawValue;
            break;
        }
        case mPC:
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));
            m_cParameter->setValue(rawValue);
            qDebug() << "KC (Position):" << rawValue;
            break;
        }
        case mPV:
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));
            m_vParameter->setValue(rawValue);
            qDebug() << "KV (Velocity):" << rawValue;
            break;
        }
        case mAC:
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));
            m_acParameter->setValue(rawValue);
            qDebug() << "AC (Angle Correction):" << rawValue;
            break;
        }
        case mArmed:
        {
            if (parsedValue.size() >= 1) {
                bool armed = (parsedValue.at(0) != 0);
                if(armed)
                    m_armedButton->setText("DisArm");
                else
                    m_armedButton->setText("Arm");
            } else {
                qDebug() << "Error: Received invalid boolean data";
            }
            break;
        }
        case mData:
        {
            break;
        }
        default:
            qDebug() << "Received unknown command:" << parsedCommand;
            break;
        }
    }

    update();
}

#if defined(Q_OS_ANDROID)
void MainWindow::requestBluetoothPermissions()
{
    QBluetoothPermission bluetoothPermission;
    bluetoothPermission.setCommunicationModes(QBluetoothPermission::Access);

    switch (qApp->checkPermission(bluetoothPermission)) {
    case Qt::PermissionStatus::Undetermined:
        qApp->requestPermission(bluetoothPermission, this,
                                [this](const QPermission &permission) {
                                    if (qApp->checkPermission(permission) == Qt::PermissionStatus::Granted) {
                                        m_bleConnection->startScan();
                                    } else {
                                        statusChanged("Bluetooth permission denied.");
                                    }
                                });
        break;
    case Qt::PermissionStatus::Granted:
        m_bleConnection->startScan();
        break;
    case Qt::PermissionStatus::Denied:
        statusChanged("Bluetooth permission denied.");
        break;
    }
}
#endif

#if defined(Q_OS_IOS)
void MainWindow::requestiOSBluetoothPermissions()
{
    if (m_iOSBluetoothInitialized) {
        return;
    }

    statusChanged("Initializing Bluetooth on iOS...");

    QTimer::singleShot(1000, this, [this]() {
        statusChanged("Starting Bluetooth scan on iOS...");
        m_iOSBluetoothInitialized = true;
        m_bleConnection->startScan();
    });
}
#endif

void MainWindow::createMessage(uint8_t msgId, uint8_t rw, QByteArray payload, QByteArray *result)
{
    uint8_t buffer[MaxPayload+8] = {'\0'};
    uint8_t command = msgId;

    int len = message.create_pack(rw, command, payload, buffer);

    for (int i = 0; i < len; i++) {
        result->append(static_cast<char>(buffer[i]));
    }
}

void MainWindow::parseMessage(QByteArray *data, uint8_t &command, QByteArray &value, uint8_t &rw)
{
    MessagePack parsedMessage;

    uint8_t* dataToParse = reinterpret_cast<uint8_t*>(data->data());
    QByteArray returnValue;
    if(message.parse(dataToParse, static_cast<uint8_t>(data->length()), &parsedMessage))
    {
        command = parsedMessage.command;
        rw = parsedMessage.rw;
        for(int i = 0; i< parsedMessage.len; i++)
        {
            value.append(static_cast<char>(parsedMessage.data[i]));
        }
    }
}

void MainWindow::requestData(uint8_t command)
{
    QByteArray payload;
    QByteArray sendData;
    createMessage(command, mRead, payload, &sendData);
    m_bleConnection->writeData(sendData);
}

void MainWindow::onConnectClicked()
{
    if(m_connectButton->text() == QString("Connect"))
    {
#if defined(Q_OS_ANDROID)
        requestBluetoothPermissions();
#elif defined(Q_OS_IOS)
        requestiOSBluetoothPermissions();
#else
        statusChanged("Starting Bluetooth scan...");
        m_bleConnection->startScan();
#endif
    }
    else
    {
        m_textStatus->clear();
        m_bleConnection->disconnectFromDevice();
    }
}

void MainWindow::onExitClicked()
{
    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        "Exit Application",
        "Are you sure you want to exit?",
        QMessageBox::Yes | QMessageBox::No
        );

    if (reply == QMessageBox::Yes) {
        QApplication::quit();
    }
}

void MainWindow::onArmedClicked()
{
    if(m_armedButton->text() == "Arm")
    {
        sendCommand(mArmed, 1);
        m_armedButton->setText("DisArm");
    }
    else
    {
        sendCommand(mDisArmed, 0);
        m_armedButton->setText("Arm");
    }
}

void MainWindow::onSpeakClicked()
{
    QByteArray data;
    data.append(m_speakEdit->text().toUtf8());
    sendString(mSpeak, data);
}

void MainWindow::onTestClicked()
{
     sendCommand(mTest, 1);
}

void MainWindow::onForwardPressed()
{
    sendCommand(mForward, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::onForwardReleased()
{
    sendCommand(mForward, 0);
}

void MainWindow::onBackwardPressed()
{
    sendCommand(mBackward, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::onBackwardReleased()
{
    sendCommand(mBackward, 0);
}

void MainWindow::onLeftPressed()
{
    sendCommand(mLeft, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::onLeftReleased()
{
    sendCommand(mLeft, 0);
}

void MainWindow::onRightPressed()
{
    sendCommand(mRight, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::onRightReleased()
{
    sendCommand(mRight, 0);
}

void MainWindow::onPValueChanged(float value)
{
    sendFloatCommand(mPP, value);
    qDebug() << "Sending KP value:" << value;
}

void MainWindow::onDValueChanged(float value)
{
    sendFloatCommand(mPD, value);
    qDebug() << "Sending KD value:" << value;
}

void MainWindow::onCValueChanged(float value)
{
    sendFloatCommand(mPC, value);
    qDebug() << "Sending KC value:" << value;
}

void MainWindow::onVValueChanged(float value)
{
    sendFloatCommand(mPV, value);
    qDebug() << "Sending KV value:" << value;
}

void MainWindow::onACValueChanged(float value)
{
    sendFloatCommand(mAC, value);
    qDebug() << "Sending AC value:" << value;
}

void MainWindow::sendCommand(uint8_t command, uint8_t value)
{
    QByteArray payload;
    payload.resize(1);
    payload[0] = static_cast<char>(value);

    QByteArray message;
    createMessage(command, mWrite, payload, &message);

    m_bleConnection->writeData(message);
}

void MainWindow::sendFloatCommand(uint8_t command, float value)
{
    QByteArray payload(sizeof(float), 0);
    const char* valueBytes = reinterpret_cast<const char*>(&value);

    for (size_t i = 0; i < sizeof(float); i++) {
        payload[i] = valueBytes[i];
    }

    QByteArray message;
    createMessage(command, mWrite, payload, &message);

    m_bleConnection->writeData(message);
}

void MainWindow::sendString(uint8_t command, QByteArray value)
{
    QByteArray message;
    createMessage(command, mWrite, value, &message);

    m_bleConnection->writeData(message);
}
