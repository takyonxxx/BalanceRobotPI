#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    remoteConstant = 50;

    initButtons();

    setWindowTitle(tr("BalanceRobot Remote Control"));

    // Apply common styles with proper scaling for different devices
    setupCommonStyles();

    m_bleConnection = new BluetoothClient();

    // Bluetooth bağlantıları
    connect(m_bleConnection, &BluetoothClient::statusChanged, this, &MainWindow::statusChanged);
    connect(m_bleConnection, &BluetoothClient::changedState, this, &MainWindow::changedState);

    // Buton bağlantıları
    connect(ui->m_pBConnect, &QPushButton::clicked, this, &MainWindow::on_ConnectClicked);
    connect(ui->m_pBForward, &QPushButton::pressed, this, &MainWindow::on_ForwardPressed);
    connect(ui->m_pBForward, &QPushButton::released, this, &MainWindow::on_ForwardReleased);
    connect(ui->m_pBBackward, &QPushButton::pressed, this, &MainWindow::on_BackwardPressed);
    connect(ui->m_pBBackward, &QPushButton::released, this, &MainWindow::on_BackwardReleased);
    connect(ui->m_pBLeft, &QPushButton::pressed, this, &MainWindow::on_LeftPressed);
    connect(ui->m_pBLeft, &QPushButton::released, this, &MainWindow::on_LeftReleased);
    connect(ui->m_pBRight, &QPushButton::pressed, this, &MainWindow::on_RightPressed);
    connect(ui->m_pBRight, &QPushButton::released, this, &MainWindow::on_RightReleased);
    connect(ui->m_pBExit, &QPushButton::clicked, this, &MainWindow::on_Exit);

    // Slider bağlantıları
    connect(ui->scrollP, &QSlider::valueChanged, this, &MainWindow::on_scrollP_valueChanged);
    connect(ui->scrollD, &QSlider::valueChanged, this, &MainWindow::on_scrollD_valueChanged);
    connect(ui->scrollC, &QSlider::valueChanged, this, &MainWindow::on_scrollC_valueChanged);
    connect(ui->scrollV, &QSlider::valueChanged, this, &MainWindow::on_scrollV_valueChanged);
    connect(ui->scrollAC, &QSlider::valueChanged, this, &MainWindow::on_scrollAC_valueChanged);

    connect(ui->scrollP, &QSlider::sliderReleased, this, &MainWindow::on_scrollP_sliderReleased);
    connect(ui->scrollD, &QSlider::sliderReleased, this, &MainWindow::on_scrollD_sliderReleased);
    connect(ui->scrollC, &QSlider::sliderReleased, this, &MainWindow::on_scrollC_sliderReleased);
    connect(ui->scrollV, &QSlider::sliderReleased, this, &MainWindow::on_scrollV_sliderReleased);
    connect(ui->scrollAC, &QSlider::sliderReleased, this, &MainWindow::on_scrollAC_sliderReleased);

    statusChanged("No Device Connected.");
}

#if defined(Q_OS_IOS)
void MainWindow::requestiOSBluetoothPermissions()
{
    if (m_iOSBluetoothInitialized) {
        // Don't initialize multiple times
        return;
    }

    statusChanged("Initializing Bluetooth on iOS...");

    // iOS requires Core Bluetooth framework permissions
    // We need to ensure the Bluetooth manager is properly initialized before scanning

    // Use timer to ensure proper initialization sequence
    QTimer::singleShot(1000, this, [this]() {
        statusChanged("Starting Bluetooth scan on iOS...");

        // Set flag to avoid multiple initialization
        m_iOSBluetoothInitialized = true;

        // Start the scan which will trigger the permission dialog
        m_bleConnection->startScan();
    });
}
#endif

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
                                        statusChanged("Bluetooth permission granted. Starting scan...");
                                        m_bleConnection->startScan();
                                    } else {
                                        statusChanged("Bluetooth permission denied. Cannot proceed.");
                                    }
                                });
        break;
    case Qt::PermissionStatus::Granted:
        statusChanged("Bluetooth permission already granted. Starting scan...");
        m_bleConnection->startScan();
        break;
    case Qt::PermissionStatus::Denied:
        statusChanged("Bluetooth permission denied. Please enable in Settings.");
        break;
    }
}
#endif

void MainWindow::setupCommonStyles()
{
    // Determine if we're on Android and set scale factor accordingly
#ifdef Q_OS_ANDROID
    // Higher scale factor for Android
    m_scaleFactor = 1.25;
#else
    // Normal scale for desktop
    m_scaleFactor = 1.0;
#endif

    // Apply styles to text status area
    ui->m_textStatus->setStyleSheet("color: #cccccc; background-color: #003333;");
    ui->m_textStatus->setFont(getScaledFont(12));

    // Apply styles to labels
    QString labelStyle = getLabelStyle("#FF4633");
    ui->labelP->setStyleSheet(labelStyle);
    ui->labelD->setStyleSheet(labelStyle);
    ui->labelC->setStyleSheet(labelStyle);
    ui->labelV->setStyleSheet(labelStyle);
    ui->labelAC->setStyleSheet(labelStyle);

    // Apply style to line edit
    ui->lineEdit_Speak->setStyleSheet("color: #ffffff; background-color: #FF4633;");
    ui->lineEdit_Speak->setFont(getScaledFont(18));

    // Apply styles to control buttons
    ui->m_pBForward->setStyleSheet("color: #ffffff; background-color: transparent;");
    ui->m_pBBackward->setStyleSheet("color: #ffffff; background-color: transparent;");
    ui->m_pBLeft->setStyleSheet("color: #ffffff; background-color: transparent;");
    ui->m_pBRight->setStyleSheet("color: #ffffff; background-color: transparent;");

    ui->m_pBForward->setMaximumHeight(30);
    ui->m_pBBackward->setMaximumHeight(30);

    // Apply styles to action buttons
    QString blueButtonStyle = getButtonStyle("#336699");
    QString redButtonStyle = getButtonStyle("#900C3F");

    ui->m_pBConnect->setStyleSheet(blueButtonStyle);
    ui->m_pBExit->setStyleSheet(blueButtonStyle);
    ui->m_pBSpeak->setStyleSheet(redButtonStyle);
    ui->m_pBArmed->setStyleSheet(blueButtonStyle);

    // Set scaled fonts for labels
    ui->labelP->setFont(getScaledFont(18));
    ui->labelD->setFont(getScaledFont(18));
    ui->labelC->setFont(getScaledFont(18));
    ui->labelV->setFont(getScaledFont(18));
    ui->labelAC->setFont(getScaledFont(18));

    // Set scaled fonts for buttons
    ui->m_pBConnect->setFont(getScaledFont(24, true));
    ui->m_pBExit->setFont(getScaledFont(24, true));
    ui->m_pBSpeak->setFont(getScaledFont(24, true));
    ui->m_pBArmed->setFont(getScaledFont(24, true));

    // Setup scaled icons for direction buttons
    setupScaledIcons();
}

QFont MainWindow::getScaledFont(int baseSize, bool isBold)
{
    QFont font;
    font.setPointSize(static_cast<int>(baseSize * m_scaleFactor));
    if (isBold) {
        font.setBold(true);
    }
    return font;
}

QString MainWindow::getButtonStyle(const QString &bgColor)
{
    return QString("color: #ffffff; background-color: %1;").arg(bgColor);
}

QString MainWindow::getLabelStyle(const QString &bgColor)
{
    return QString("color: #ffffff; background-color: %1;").arg(bgColor);
}

void MainWindow::setupScaledIcons()
{
    // Calculate icon size based on scale factor
    int iconWidth = static_cast<int>(96 * m_scaleFactor);
    int iconHeight = static_cast<int>(64 * m_scaleFactor);

    QSize iconSize(iconWidth, iconHeight);

    // Forward button
    QPixmap pixmapf(":/icons/forward.png");
    QIcon ForwardIcon(pixmapf.scaled(iconSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->m_pBForward->setIcon(ForwardIcon);
    ui->m_pBForward->setIconSize(iconSize);
    ui->m_pBForward->setFixedSize(iconSize);

    // Backward button
    QPixmap pixmapb(":/icons/back.png");
    QIcon BackwardIcon(pixmapb.scaled(iconSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->m_pBBackward->setIcon(BackwardIcon);
    ui->m_pBBackward->setIconSize(iconSize);
    ui->m_pBBackward->setFixedSize(iconSize);

    // Left button
    QPixmap pixmapl(":/icons/left.png");
    QIcon LeftIcon(pixmapl.scaled(iconSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->m_pBLeft->setIcon(LeftIcon);
    ui->m_pBLeft->setIconSize(iconSize);
    ui->m_pBLeft->setFixedSize(iconSize);

    // Right button
    QPixmap pixmapr(":/icons/right.png");
    QIcon RightIcon(pixmapr.scaled(iconSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    ui->m_pBRight->setIcon(RightIcon);
    ui->m_pBRight->setIconSize(iconSize);
    ui->m_pBRight->setFixedSize(iconSize);
}

void MainWindow::changedState(BluetoothClient::bluetoothleState state){

    switch(state){

    case BluetoothClient::Scanning:
    {
        statusChanged("Searching for low energy devices...");
        break;
    }
    case BluetoothClient::ScanFinished:
    {
        break;
    }

    case BluetoothClient::Connecting:
    {
        break;
    }
    case BluetoothClient::Connected:
    {
        ui->m_pBConnect->setText("Disconnect");
        connect(m_bleConnection, SIGNAL(newData(QByteArray)), this, SLOT(DataHandler(QByteArray)));

        break;
    }
    case BluetoothClient::DisConnected:
    {
        statusChanged("Device disconnected.");
        ui->m_pBConnect->setEnabled(true);
        ui->m_pBConnect->setText("Connect");
        ui->scrollAC->setValue(0);
        ui->scrollV->setValue(0);
        ui->scrollD->setValue(0);
        ui->scrollC->setValue(0);
        ui->scrollP->setValue(0);
        break;
    }
    case BluetoothClient::ServiceFound:
    {
        break;
    }
    case BluetoothClient::AcquireData:
    {
        requestData(mPP);
        requestData(mPD);
        requestData(mPC);
        requestData(mPV);
        requestData(mAC);
        requestData(mArmed);
        break;
    }
    case BluetoothClient::Error:
    {
        ui->m_textStatus->clear();
        break;
    }
    default:
        //nothing for now
        break;
    }
}

void MainWindow::DataHandler(QByteArray data)
{
    uint8_t parsedCommand;
    uint8_t rw;
    QByteArray parsedValue;
    parseMessage(&data, parsedCommand, parsedValue, rw);

    if(rw == mWrite)
    {
        // Bu değerler için PID parametrelerinin min ve max değerlerini tanımlayalım
        const float KP_MIN = 0.0f;    // Angle gain (min)
        const float KP_MAX = 0.5f;    // Angle gain (max)
        const float KD_MIN = 0.0f;    // Angle velocity gain (min)
        const float KD_MAX = 10.0f;   // Angle velocity gain (max)
        const float KC_MIN = 0.0f;    // Position feedback gain (min)
        const float KC_MAX = 0.001f;  // Position feedback gain (max)
        const float KV_MIN = 0.0f;    // Velocity feedback gain (min)
        const float KV_MAX = 0.05f;   // Velocity feedback gain (max)
        const float AC_MIN = 0.0f;    // Angle correction (min)
        const float AC_MAX = 5.0f;    // Angle correction (max)

        switch(parsedCommand) {
        case mPP: // KP - Angle gain
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));

            // KP değerini 0-100 slider ölçeğine dönüştür
            int sliderValue = ((rawValue - KP_MIN) / (KP_MAX - KP_MIN)) * 100;
            sliderValue = qBound(0, sliderValue, 100); // 0-100 aralığında sınırla

            ui->scrollP->setValue(sliderValue);
            ui->labelP->setText(QString("%1").arg(rawValue, 0, 'f', 3));
            qDebug() << "KP (Proportional):" << rawValue << "Slider:" << sliderValue;
            break;
        }
        case mPD: // KD - Angle velocity gain
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));

            // KD değerini 0-100 slider ölçeğine dönüştür
            int sliderValue = ((rawValue - KD_MIN) / (KD_MAX - KD_MIN)) * 100;
            sliderValue = qBound(0, sliderValue, 100);

            ui->scrollD->setValue(sliderValue);
            ui->labelD->setText(QString("%1").arg(rawValue, 0, 'f', 3));
            qDebug() << "KD (Derivative):" << rawValue << "Slider:" << sliderValue;
            break;
        }
        case mPC: // KC - Position feedback gain
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));

            // KC değerini 0-100 slider ölçeğine dönüştür
            int sliderValue = ((rawValue - KC_MIN) / (KC_MAX - KC_MIN)) * 100;
            sliderValue = qBound(0, sliderValue, 100);

            ui->scrollC->setValue(sliderValue);
            ui->labelC->setText(QString("%1").arg(rawValue, 0, 'f', 6)); // Daha fazla ondalık basamak
            qDebug() << "KC (Position):" << rawValue << "Slider:" << sliderValue;
            break;
        }
        case mPV:
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));

            // SD değerini 0-100 slider ölçeğine dönüştür
            int sliderValue = ((rawValue - KV_MIN) / (KV_MAX - KV_MIN)) * 100;
            sliderValue = qBound(0, sliderValue, 100);

            ui->scrollV->setValue(sliderValue);
            ui->labelV->setText(QString("%1").arg(rawValue, 0, 'f', 2));
            qDebug() << "SD (Speed Diff):" << rawValue << "Slider:" << sliderValue;
            break;
        }
        case mAC: // Angle correction
        {
            float rawValue;
            memcpy(&rawValue, parsedValue.constData(), sizeof(float));

            // AC değerini 0-100 slider ölçeğine dönüştür
            int sliderValue = ((rawValue - AC_MIN) / (AC_MAX - AC_MIN)) * 100;
            sliderValue = qBound(0, sliderValue, 100);

            ui->scrollAC->setValue(sliderValue);
            ui->labelAC->setText(QString("%1").arg(rawValue, 0, 'f', 2));
            qDebug() << "AC (Angle Correction):" << rawValue << "Slider:" << sliderValue;
            break;
        }
        case mArmed:
        {
            // Boolean değeri işle
            if (parsedValue.size() >= 1) {
                bool armed = (parsedValue.at(0) != 0);
                if(armed)
                    ui->m_pBArmed->setText("DisArm");
                else
                    ui->m_pBArmed->setText("Arm");
            } else {
                qDebug() << "Error: Received invalid boolean data";
            }
            break;
        }
        case mData:
        {
            ui->m_textStatus->append(parsedValue.toStdString().c_str());
            break;
        }
        default:
            qDebug() << "Received unknown command:" << parsedCommand;
            break;
        }
    }
    update();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::initButtons(){
    /* Init Buttons */
    ui->m_pBConnect->setText("Connect");
}

void MainWindow::statusChanged(const QString &status)
{
    ui->m_textStatus->append(status);
}

void MainWindow::on_ConnectClicked()
{
    if(ui->m_pBConnect->text() == QString("Connect"))
    {
#if defined(Q_OS_ANDROID)
        requestBluetoothPermissions();
#elif defined(Q_OS_IOS)
        requestiOSBluetoothPermissions();
#else
        // Desktop platforms - typically don't need special permission handling
        statusChanged("Starting Bluetooth scan...");
        m_bleConnection->startScan();
#endif
    }
    else
    {
        ui->m_textStatus->clear();
        m_bleConnection->disconnectFromDevice();
    }
}

void MainWindow::createMessage(uint8_t msgId, uint8_t rw, QByteArray payload, QByteArray *result)
{
    uint8_t buffer[MaxPayload+8] = {'\0'};
    uint8_t command = msgId;

    int len = message.create_pack(rw , command , payload, buffer);

    for (int i = 0; i < len; i++)
    {
        result->append(static_cast<char>(buffer[i]));
    }
}

void MainWindow::parseMessage(QByteArray *data, uint8_t &command, QByteArray &value,  uint8_t &rw)
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

void MainWindow::sendCommand(uint8_t command, uint8_t value)
{
    QByteArray payload;
    payload.resize(1);

    // Assign the value to the first element of the payload
    payload[0] = static_cast<char>(value);

    // Create the message and send it
    QByteArray message;
    createMessage(command, mWrite, payload, &message);

    m_bleConnection->writeData(message);
}

void MainWindow::sendFloatCommand(uint8_t command, float value)
{
    // Float değeri baytlara dönüştür
    QByteArray payload(sizeof(float), 0);
    const char* valueBytes = reinterpret_cast<const char*>(&value);

    // Baytları payload'a kopyala
    for (size_t i = 0; i < sizeof(float); i++) {
        payload[i] = valueBytes[i];
    }

    // Mesajı oluştur ve gönder
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

void MainWindow::on_scrollP_valueChanged(int value)
{
    // KP için min-max değerler
    const float KP_MIN = 0.0f;
    const float KP_MAX = 0.5f;

    // Slider değerini gerçek KP değerine dönüştür
    float realValue = KP_MIN + (value / 100.0f) * (KP_MAX - KP_MIN);

    // GUI'yi güncelle
    ui->labelP->setText(QString("%1").arg(realValue, 0, 'f', 3));

    // Not: Float değeri burada gönderilmez, sadece sliderReleased'de gönderilir
}

void MainWindow::on_scrollD_valueChanged(int value)
{
    // KD için min-max değerler
    const float KD_MIN = 0.0f;
    const float KD_MAX = 10.0f;  // Düzeltildi: KD için daha uygun bir aralık

    // Slider değerini gerçek KD değerine dönüştür
    float realValue = KD_MIN + (value / 100.0f) * (KD_MAX - KD_MIN);

    // GUI'yi güncelle
    ui->labelD->setText(QString("%1").arg(realValue, 0, 'f', 3));

    // Not: Float değeri burada gönderilmez
}

void MainWindow::on_scrollC_valueChanged(int value)
{
    // KC için min-max değerler
    const float KC_MIN = 0.0f;
    const float KC_MAX = 0.001f;  // KC için uygun aralık

    // Slider değerini gerçek KC değerine dönüştür
    float realValue = KC_MIN + (value / 100.0f) * (KC_MAX - KC_MIN);

    // GUI'yi güncelle
    ui->labelC->setText(QString("%1").arg(realValue, 0, 'f', 6));

    // Not: Float değeri burada gönderilmez
}

void MainWindow::on_scrollV_valueChanged(int value)
{
    // KV için min-max değerler
    const float KV_MIN = 0.0f;
    const float KV_MAX = 0.05f;  // KV için uygun aralık

    // Slider değerini gerçek KV değerine dönüştür
    float realValue = KV_MIN + (value / 100.0f) * (KV_MAX - KV_MIN);

    // GUI'yi güncelle
    ui->labelV->setText(QString("%1").arg(realValue, 0, 'f', 4));

    // Not: Float değeri burada gönderilmez
}

void MainWindow::on_scrollAC_valueChanged(int value)
{
    // AC için min-max değerler
    const float AC_MIN = 0.0f;
    const float AC_MAX = 5.0f;

    // Slider değerini gerçek AC değerine dönüştür
    float realValue = AC_MIN + (value / 100.0f) * (AC_MAX - AC_MIN);

    // GUI'yi güncelle
    ui->labelAC->setText(QString("%1").arg(realValue, 0, 'f', 2));

    // Not: Float değeri burada gönderilmez
}

void MainWindow::on_scrollP_sliderReleased()
{
    int value = ui->scrollP->value();
    const float KP_MIN = 0.0f;
    const float KP_MAX = 0.5f;
    float realValue = KP_MIN + (value / 100.0f) * (KP_MAX - KP_MIN);

    // Float değeri gönder
    sendFloatCommand(mPP, realValue);
    qDebug() << "KP value sent:" << realValue;
}

void MainWindow::on_scrollD_sliderReleased()
{
    int value = ui->scrollD->value();
    const float KD_MIN = 0.0f;
    const float KD_MAX = 10.0f;
    float realValue = KD_MIN + (value / 100.0f) * (KD_MAX - KD_MIN);

    // Float değeri gönder
    sendFloatCommand(mPD, realValue);  // Düzeltildi: mPP yerine mPD
    qDebug() << "KD value sent:" << realValue;
}

void MainWindow::on_scrollC_sliderReleased()
{
    int value = ui->scrollC->value();
    const float KC_MIN = 0.0f;
    const float KC_MAX = 0.001f;
    float realValue = KC_MIN + (value / 100.0f) * (KC_MAX - KC_MIN);

    // Float değeri gönder
    sendFloatCommand(mPC, realValue);  // Düzeltildi: mPP yerine mPC
    qDebug() << "KC value sent:" << realValue;
}

void MainWindow::on_scrollV_sliderReleased()
{
    int value = ui->scrollV->value();
    const float KV_MIN = 0.0f;
    const float KV_MAX = 0.05f;
    float realValue = KV_MIN + (value / 100.0f) * (KV_MAX - KV_MIN);

    // Float değeri gönder
    sendFloatCommand(mPV, realValue);  // Düzeltildi: mPP yerine mPV
    qDebug() << "KV value sent:" << realValue;
}

void MainWindow::on_scrollAC_sliderReleased()
{
    int value = ui->scrollAC->value();
    const float AC_MIN = 0.0f;
    const float AC_MAX = 5.0f;
    float realValue = AC_MIN + (value / 100.0f) * (AC_MAX - AC_MIN);

    // Float değeri gönder
    sendFloatCommand(mAC, realValue);  // Düzeltildi: mPP yerine mAC
    qDebug() << "AC value sent:" << realValue;
}

void MainWindow::on_Exit()
{
    exit(0);
}

void MainWindow::on_ForwardPressed()
{
    sendCommand(mForward, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::on_ForwardReleased()
{
    sendCommand(mForward, 0);
}

void MainWindow::on_BackwardPressed()
{
    sendCommand(mBackward, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::on_BackwardReleased()
{
    sendCommand(mBackward, 0);
}

void MainWindow::on_RightPressed()
{
    sendCommand(mRight, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::on_RightReleased()
{
    sendCommand(mRight, 0);
}

void MainWindow::on_LeftPressed()
{
    sendCommand(mLeft, static_cast<uint8_t>(remoteConstant));
}

void MainWindow::on_LeftReleased()
{
    sendCommand(mLeft, 0);
}

void MainWindow::on_m_pBSpeak_clicked()
{
    QByteArray data;
    data.append(QString(ui->lineEdit_Speak->text()).toUtf8());
    sendString(mSpeak, data);
}

void MainWindow::on_m_pBFormat_clicked()
{
    ui->lineEdit_Speak->setText("espeak -vtr+f6");
}

void MainWindow::on_m_pBArmed_clicked()
{
    if(ui->m_pBArmed->text() == "Arm")
    {
        sendCommand(mArmed, 0);
        ui->m_pBArmed->setText("DisArm");
    }
    else
    {
        sendCommand(mDisArmed, 0);
        ui->m_pBArmed->setText("Arm");
    }
}

