/********************************************************************************
** Form generated from reading UI file 'my_plugin.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MY_PLUGIN_H
#define UI_MY_PLUGIN_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MyPluginWidget
{
public:
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QLabel *BateryLabel;
    QPushButton *EmergencyButton;
    QSpacerItem *verticalSpacer_2;
    QPushButton *LandButton;
    QPushButton *ResetPoseButton;
    QPushButton *TakeOffButton;
    QSpacerItem *verticalSpacer_1;
    QSpacerItem *verticalSpacer;
    QLabel *BatteryPercentageLabel;
    QLineEdit *addEdit;
    QPushButton *addButton;
    QComboBox *DroneSelect;
    QPushButton *removeButton;

    void setupUi(QWidget *MyPluginWidget)
    {
        if (MyPluginWidget->objectName().isEmpty())
            MyPluginWidget->setObjectName(QString::fromUtf8("MyPluginWidget"));
        MyPluginWidget->resize(452, 413);
        gridLayout_2 = new QGridLayout(MyPluginWidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        BateryLabel = new QLabel(MyPluginWidget);
        BateryLabel->setObjectName(QString::fromUtf8("BateryLabel"));
        BateryLabel->setLocale(QLocale(QLocale::English, QLocale::Belgium));
        BateryLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(BateryLabel, 8, 0, 1, 1);

        EmergencyButton = new QPushButton(MyPluginWidget);
        EmergencyButton->setObjectName(QString::fromUtf8("EmergencyButton"));
        EmergencyButton->setLocale(QLocale(QLocale::English, QLocale::Belgium));

        gridLayout->addWidget(EmergencyButton, 3, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 2, 0, 1, 2);

        LandButton = new QPushButton(MyPluginWidget);
        LandButton->setObjectName(QString::fromUtf8("LandButton"));
        LandButton->setLocale(QLocale(QLocale::English, QLocale::Belgium));

        gridLayout->addWidget(LandButton, 6, 1, 1, 1);

        ResetPoseButton = new QPushButton(MyPluginWidget);
        ResetPoseButton->setObjectName(QString::fromUtf8("ResetPoseButton"));
        ResetPoseButton->setLocale(QLocale(QLocale::English, QLocale::Belgium));

        gridLayout->addWidget(ResetPoseButton, 3, 1, 1, 1);

        TakeOffButton = new QPushButton(MyPluginWidget);
        TakeOffButton->setObjectName(QString::fromUtf8("TakeOffButton"));
        TakeOffButton->setLocale(QLocale(QLocale::English, QLocale::Belgium));

        gridLayout->addWidget(TakeOffButton, 6, 0, 1, 1);

        verticalSpacer_1 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_1, 4, 0, 1, 2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 7, 0, 1, 2);

        BatteryPercentageLabel = new QLabel(MyPluginWidget);
        BatteryPercentageLabel->setObjectName(QString::fromUtf8("BatteryPercentageLabel"));
        BatteryPercentageLabel->setLocale(QLocale(QLocale::English, QLocale::Belgium));
        BatteryPercentageLabel->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(BatteryPercentageLabel, 8, 1, 1, 1);

        addEdit = new QLineEdit(MyPluginWidget);
        addEdit->setObjectName(QString::fromUtf8("addEdit"));
        QSizePolicy sizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(addEdit->sizePolicy().hasHeightForWidth());
        addEdit->setSizePolicy(sizePolicy);

        gridLayout->addWidget(addEdit, 0, 0, 1, 1);

        addButton = new QPushButton(MyPluginWidget);
        addButton->setObjectName(QString::fromUtf8("addButton"));
        sizePolicy.setHeightForWidth(addButton->sizePolicy().hasHeightForWidth());
        addButton->setSizePolicy(sizePolicy);

        gridLayout->addWidget(addButton, 0, 1, 1, 1);

        DroneSelect = new QComboBox(MyPluginWidget);
        DroneSelect->setObjectName(QString::fromUtf8("DroneSelect"));
        DroneSelect->setLocale(QLocale(QLocale::English, QLocale::Belgium));

        gridLayout->addWidget(DroneSelect, 1, 0, 1, 1);

        removeButton = new QPushButton(MyPluginWidget);
        removeButton->setObjectName(QString::fromUtf8("removeButton"));

        gridLayout->addWidget(removeButton, 1, 1, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);


        retranslateUi(MyPluginWidget);

        QMetaObject::connectSlotsByName(MyPluginWidget);
    } // setupUi

    void retranslateUi(QWidget *MyPluginWidget)
    {
        MyPluginWidget->setWindowTitle(QApplication::translate("MyPluginWidget", "ucl_drone_gui", 0, QApplication::UnicodeUTF8));
        BateryLabel->setText(QApplication::translate("MyPluginWidget", "Battery", 0, QApplication::UnicodeUTF8));
        EmergencyButton->setText(QApplication::translate("MyPluginWidget", "Emergency", 0, QApplication::UnicodeUTF8));
        LandButton->setText(QApplication::translate("MyPluginWidget", "Land", 0, QApplication::UnicodeUTF8));
        ResetPoseButton->setText(QApplication::translate("MyPluginWidget", "Reset Pose", 0, QApplication::UnicodeUTF8));
        TakeOffButton->setText(QApplication::translate("MyPluginWidget", "Take off", 0, QApplication::UnicodeUTF8));
        BatteryPercentageLabel->setText(QApplication::translate("MyPluginWidget", "N/A%", 0, QApplication::UnicodeUTF8));
        addButton->setText(QApplication::translate("MyPluginWidget", "Add", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        DroneSelect->setToolTip(QApplication::translate("MyPluginWidget", "<html><head/><body><p>Drone group name</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        removeButton->setText(QApplication::translate("MyPluginWidget", "Remove", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MyPluginWidget: public Ui_MyPluginWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MY_PLUGIN_H
