
QT += core
QT -= gui
CONFIG += c++17 cmdline
CONFIG -= app_bundle

TARGET = BalanceRobot
TEMPLATE = app

SOURCES += \
        i2cdev.cpp \
        main.cpp \
        mpu6050.cpp \
        robotcontrol.cpp

HEADERS += \
    i2cdev.h \
    mpu6050.h \
    robotcontrol.h

INCLUDEPATH += /usr/local/include

# Libraries needed
LIBS += -lpigpio -lpthread -lrt -li2c

# Default rules for deployment
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# sudo apt install qt6-base-dev qt6-base-dev-tools
# sudo ln -sf /usr/lib/qt6/bin/qmake6 /usr/local/bin/qmake
# sudo apt install libi2c-dev i2c-tools
# sudo apt-get install libpigpio-dev
# git clone https://github.com/WiringPi/WiringPi.git
# cd WiringPi
# sudo ./build
# nmap -sn 192.168.1.0/24
# sudo raspi-config enable i2c
# sudo nmcli connection add type wifi con-name "SSID" ifname wlan0 ssid "SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "password"
# sudo nmcli connection modify "SSID" connection.autoconnect yes
# nmcli connection show
# nmcli connection show SSID | grep -E 'autoconnect|ssid|psk'
