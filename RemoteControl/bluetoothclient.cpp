#include "bluetoothclient.h"
#include <QTimer>
#include <QBluetoothLocalDevice>

BluetoothClient::BluetoothClient() :
    m_control(nullptr),
    m_service(nullptr),
    m_bFoundUARTService(false),
    m_state(bluetoothleState::Idle)
{
    /* 1 Step: Bluetooth LE Device Discovery */
    m_deviceDiscoveryAgent = new QBluetoothDeviceDiscoveryAgent(this);
    m_deviceDiscoveryAgent->setLowEnergyDiscoveryTimeout(60000);

    /* Device Discovery Initialization */
    connect(m_deviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::deviceDiscovered,
            this, &BluetoothClient::addDevice);
    connect(m_deviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::errorOccurred,
            this, &BluetoothClient::deviceScanError);
    connect(m_deviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::finished,
            this, &BluetoothClient::scanFinished);
    connect(m_deviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::canceled,
            this, &BluetoothClient::scanFinished);
}

BluetoothClient::~BluetoothClient(){
    if(current_device)
        delete current_device;
}

void BluetoothClient::getDeviceList(QList<QString> &qlDevices){
    if(m_state == bluetoothleState::ScanFinished && current_device)
    {
        qlDevices.append(current_device->getName());
    }
    else
    {
        qlDevices.clear();
    }
}

void BluetoothClient::addDevice(const QBluetoothDeviceInfo &device)
{
    /* Is it a LE Bluetooth device? */
    if (device.coreConfigurations() & QBluetoothDeviceInfo::LowEnergyCoreConfiguration)
    {
        if(device.name().isEmpty()) return;

        if(device.name().startsWith("Balance"))
        {
            current_device = new DeviceInfo(device);
            m_deviceDiscoveryAgent->stop();
            QTimer::singleShot(500, this, [this]() {
                emit m_deviceDiscoveryAgent->finished();
                startConnect(0);
            });
        }
    }
}

void BluetoothClient::scanFinished()
{
    setState(ScanFinished);
}

void BluetoothClient::deviceScanError(QBluetoothDeviceDiscoveryAgent::Error error)
{
    QString info;
    if (error == QBluetoothDeviceDiscoveryAgent::PoweredOffError)
        info = "The Bluetooth adaptor is powered off.";
    else if (error == QBluetoothDeviceDiscoveryAgent::InputOutputError)
        info = "Writing or reading error.";
    else {
        static QMetaEnum qme = m_deviceDiscoveryAgent->metaObject()->enumerator(
            m_deviceDiscoveryAgent->metaObject()->indexOfEnumerator("Error"));
        info = "Error: " + QLatin1String(qme.valueToKey(error));
    }

    setState(Error);
}

void BluetoothClient::startScan(){
    setState(Scanning);
    current_device = nullptr;
    m_bFoundUARTService = false;

    // Set up the discovery agent to only look for LE devices
    // Use useLowEnergyDiscoveryTimeout instead of setDiscoveryMethods
    m_deviceDiscoveryAgent->setLowEnergyDiscoveryTimeout(60000);

    // Start the discovery
    m_deviceDiscoveryAgent->start(QBluetoothDeviceDiscoveryAgent::LowEnergyMethod);
}

void BluetoothClient::startConnect(int i){
    m_qvMeasurements.clear();

    if (m_control) {
        m_control->disconnectFromDevice();
        delete m_control;
        m_control = nullptr;
    }

    /* 2 Step: QLowEnergyController */
    m_control = QLowEnergyController::createCentral(current_device->getDevice(), this);
    m_control->setRemoteAddressType(QLowEnergyController::RandomAddress);

    // Ensure device is in connectable mode
    QBluetoothLocalDevice *localDevice = new QBluetoothLocalDevice();
    localDevice->setHostMode(QBluetoothLocalDevice::HostConnectable);
    delete localDevice;

    connect(m_control, &QLowEnergyController::errorOccurred, this, &BluetoothClient::errorOccurred);
    connect(m_control, &QLowEnergyController::connected, this, &BluetoothClient::deviceConnected);
    connect(m_control, &QLowEnergyController::disconnected, this, &BluetoothClient::deviceDisconnected);
    connect(m_control, &QLowEnergyController::serviceDiscovered, this, &BluetoothClient::serviceDiscovered);
    connect(m_control, &QLowEnergyController::discoveryFinished, this, &BluetoothClient::serviceScanDone);

    /* Start connecting to device */
    m_control->connectToDevice();
    setState(Connecting);
}

void BluetoothClient::setService_name(const QString &newService_name)
{
    m_service_name = newService_name;
}

void BluetoothClient::serviceDiscovered(const QBluetoothUuid &gatt)
{
    QString discoveredUuid = gatt.toString().remove("{").remove("}").toLower();
    QString targetUuid = QString(SCANPARAMETERSUUID).toLower();

    if (discoveredUuid.contains(targetUuid) || targetUuid.contains(discoveredUuid)) {
        m_bFoundUARTService = true;
        current_gatt = gatt;
    }
}

void BluetoothClient::serviceScanDone()
{
    delete m_service;
    m_service = nullptr;

    if(m_bFoundUARTService)
    {
        m_service = m_control->createServiceObject(current_gatt, this);
    }

    if(!m_service)
    {
        disconnectFromDevice();
        setState(DisConnected);
        return;
    }

    /* 3 Step: Service Discovery */
    connect(m_service, SIGNAL(stateChanged(QLowEnergyService::ServiceState)),
            this, SLOT(serviceStateChanged(QLowEnergyService::ServiceState)));
    connect(m_service, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
            this, SLOT(updateData(QLowEnergyCharacteristic,QByteArray)));
    connect(m_service, SIGNAL(descriptorWritten(QLowEnergyDescriptor,QByteArray)),
            this, SLOT(confirmedDescriptorWrite(QLowEnergyDescriptor,QByteArray)));

    m_service->discoverDetails();
    setState(ServiceFound);
}

void BluetoothClient::disconnectFromDevice()
{
    if (m_control) {
        m_control->disconnectFromDevice();
    } else {
        setState(DisConnected);
    }
}

void BluetoothClient::deviceDisconnected()
{
    delete m_service;
    m_service = nullptr;
    setState(DisConnected);
}

void BluetoothClient::deviceConnected()
{
    m_control->discoverServices();
    setState(Connected);
}

void BluetoothClient::errorOccurred(QLowEnergyController::Error newError)
{
    auto statusText = QString("Controller Error: %1").arg(newError);
    qDebug() << statusText;
    // Add a recovery attempt for certain errors
    if (newError == QLowEnergyController::ConnectionError) {
        QTimer::singleShot(3000, this, [this]() {
            if (current_device && m_state == Error) {
                startConnect(0);
            }
        });
    }
}

void BluetoothClient::controllerError(QLowEnergyController::Error error)
{
    QString info = QStringLiteral("Controller Error: ") + m_control->errorString();
    setState(Error);
    qDebug() << info;
}

void BluetoothClient::searchCharacteristic()
{
    if(m_service){
        foreach (QLowEnergyCharacteristic c, m_service->characteristics())
        {
            if(c.isValid())
            {
                if (c.properties() & QLowEnergyCharacteristic::WriteNoResponse ||
                    c.properties() & QLowEnergyCharacteristic::Write)
                {
                    m_writeCharacteristic = c;

                    if(c.properties() & QLowEnergyCharacteristic::WriteNoResponse)
                        m_writeMode = QLowEnergyService::WriteWithoutResponse;
                    else
                        m_writeMode = QLowEnergyService::WriteWithResponse;
                }
                if (c.properties() & QLowEnergyCharacteristic::Notify || c.properties() & QLowEnergyCharacteristic::Read)
                {
                    m_readCharacteristic = c;
                    m_notificationDescRx = c.descriptor(QBluetoothUuid::DescriptorType::ClientCharacteristicConfiguration);
                    if (m_notificationDescRx.isValid())
                    {
                        // Add a small delay before writing the descriptor
                        QTimer::singleShot(100, this, [this]() {
                            m_service->writeDescriptor(m_notificationDescRx, QByteArray::fromHex("0100"));
                        });
                    }
                }
            }
        }
    }
}

/* Slots for QLowEnergyService */
void BluetoothClient::serviceStateChanged(QLowEnergyService::ServiceState s)
{
    if (s == QLowEnergyService::ServiceDiscovered)
    {
        searchCharacteristic();
    } else if (s == QLowEnergyService::InvalidService) {
    } else if (s == QLowEnergyService::DiscoveringService) {
    }
}

void BluetoothClient::updateData(const QLowEnergyCharacteristic &c, const QByteArray &value)
{
    // Check if data is valid before processing
    if (value.isEmpty()) {
        return;
    }

    // Log the received data
    QString hexData;
    for (char byte : value) {
        hexData += QString("%1 ").arg(static_cast<unsigned char>(byte), 2, 16, QChar('0'));
    }

    // Process the data safely
    try {
        emit newData(value);
    } catch (const std::exception& e) {
    } catch (...) {
    }
}

void BluetoothClient::confirmedDescriptorWrite(const QLowEnergyDescriptor &d, const QByteArray &value)
{
    setState(AcquireData);
}

void BluetoothClient::writeData(QByteArray data)
{
    if(m_service && m_writeCharacteristic.isValid())
    {
        // Add a small delay before writing to avoid GATT timeouts
        QTimer::singleShot(50, this, [this, data]() {
            m_service->writeCharacteristic(m_writeCharacteristic, data, m_writeMode);
        });
    }
}

void BluetoothClient::setState(BluetoothClient::bluetoothleState newState)
{
    m_state = newState;

    // Add debug information about state changes
    QString stateStr;
    switch (newState) {
    case Idle: stateStr = "Idle"; break;
    case Scanning: stateStr = "Scanning"; break;
    case ScanFinished: stateStr = "Scan Finished"; break;
    case Connecting: stateStr = "Connecting"; break;
    case Connected: stateStr = "Connected"; break;
    case ServiceFound: stateStr = "Service Found"; break;
    case DisConnected: stateStr = "Disconnected"; break;
    case AcquireData: stateStr = "Ready for Data"; break;
    case Error: stateStr = "Error"; break;
    default: stateStr = "Unknown"; break;
    }

    emit changedState(newState);

    qDebug() << "Bluetooth state changed to:" << stateStr;
}

BluetoothClient::bluetoothleState BluetoothClient::getState() const {
    return m_state;
}
