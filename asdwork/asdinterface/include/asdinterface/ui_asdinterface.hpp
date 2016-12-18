#ifndef UI_ASDINTERFACE_H
#define UI_ASDINTERFACE_H

#include <QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QHeaderView>
#include <QLCDNumber>
#include <QPushButton>
#include <QWidget>
#include <QPalette>
#include <QTextEdit>

QT_BEGIN_NAMESPACE

class Ui_ASDInterface{
	public:
		QPushButton *Start;
		QPushButton *Command;
		QPushButton *Prompt;
		QPushButton *StartRecord;
		QPushButton *StopRecord;
		QPushButton *Respond;
		QPushButton *Bye;
		QPushButton *ShutDown;
		QPushButton *AngleHead;
		QPushButton *Rest;
		QPushButton *Stand;
		QPushButton *ToggleLife;
		QTextEdit *Name;
		QLCDNumber *MyClock;
		void setupUi(QWidget *ASDInterface){
			if(ASDInterface->objectName().isEmpty())
				ASDInterface->setObjectName(QString("ASDInterface"));
			int blockw = 173;
			int blockh = 40;
			int buffer = 20;

			ASDInterface->resize(600, 600);

			//QPalette Pal(palette());
 
			// set black background
			//Pal.setColor(QPalette::Background, Qt::black);
			
			Command = new QPushButton(ASDInterface);
			Command->setObjectName(QString("Command"));
			Command->setGeometry(QRect(20, 20, blockw, blockh));
			//Command->setAutoFillBackground(true);
			//Command->setPalette(Pal);
			Respond = new QPushButton(ASDInterface);
			Respond->setObjectName(QString("Respond"));
			Respond->setGeometry(QRect(blockw + buffer*2, buffer, blockw, blockh));
			Prompt = new QPushButton(ASDInterface);
			Prompt->setObjectName(QString("Prompt"));
			Prompt->setGeometry(QRect(buffer, blockh + buffer*2, blockw, blockh));
			Bye = new QPushButton(ASDInterface);
			Bye->setObjectName(QString("Bye"));
			Bye->setGeometry(QRect(blockw + buffer*2, blockh + buffer*2, blockw, blockh));

			StartRecord = new QPushButton(ASDInterface);
			StartRecord->setObjectName(QString("StartRecord"));
			StartRecord->setGeometry(QRect(blockw*2 + buffer*3, buffer, blockw, blockh));
			StopRecord = new QPushButton(ASDInterface);
			StopRecord->setObjectName(QString("StopRecord"));
			StopRecord->setGeometry(QRect(blockw*2 + buffer*3, blockh + buffer*2, blockw, blockh));

			AngleHead = new QPushButton(ASDInterface);
			AngleHead->setObjectName(QString("AngleHead"));
			AngleHead->setGeometry(QRect(blockw*2 + buffer*3, blockh*2 + buffer*3, blockw, blockh));
			Stand = new QPushButton(ASDInterface);
			Stand->setObjectName(QString("Stand"));
			Stand->setGeometry(QRect(blockw + buffer*2, blockh*2 + buffer*3, blockw, blockh));
			Rest = new QPushButton(ASDInterface);
			Rest->setObjectName(QString("Rest"));
			Rest->setGeometry(QRect(blockw + buffer*2, blockh*3 + buffer*4, blockw, blockh));
			ToggleLife = new QPushButton(ASDInterface);
			ToggleLife->setObjectName(QString("ToggleLife"));
			ToggleLife->setGeometry(QRect(blockw*2 + buffer*3, blockh*3 + buffer*4, blockw, blockh));


			Start = new QPushButton(ASDInterface);
			Start->setObjectName(QString("Start"));
			Start->setGeometry(QRect(blockw*2 + buffer*3, 600-buffer*2-blockh*2, blockw, blockh));
			ShutDown = new QPushButton(ASDInterface);
			ShutDown->setObjectName(QString("ShutDown"));
			ShutDown->setGeometry(QRect(blockw*2 + buffer*3, 600-blockh-buffer, blockw, blockh));

			Name = new QTextEdit(ASDInterface);
			Name->setObjectName(QString("Name"));
			Name->setGeometry(QRect(buffer, blockh*2 + buffer*3, blockw, blockh));

			MyClock = new QLCDNumber(ASDInterface);
			MyClock->setObjectName(QString("MyClock"));
			MyClock->setGeometry(QRect(370, 375, 201, 81));
	
			retranslateUi(ASDInterface);
	
			QMetaObject::connectSlotsByName(ASDInterface);
		}
	
		void retranslateUi(QWidget *ASDInterface){
			ASDInterface->setWindowTitle(QApplication::translate("ASDInterface", "ASDInterface", 0));

			Command->setText(QApplication::translate("ASDInterface", "Command", 0));
			Respond->setText(QApplication::translate("ASDInterface", "Respond", 0));
			Prompt->setText(QApplication::translate("ASDInterface", "Prompt", 0));
			Bye->setText(QApplication::translate("ASDInterface", "Bye", 0));

			StartRecord->setText(QApplication::translate("ASDInterface", "StartRecord", 0));
			StopRecord->setText(QApplication::translate("ASDInterface", "StopRecord", 0));

			AngleHead->setText(QApplication::translate("ASDInterface", "AngleHead", 0));
			Rest->setText(QApplication::translate("ASDInterface", "Rest", 0));
			Stand->setText(QApplication::translate("ASDInterface", "Stand", 0));
			ToggleLife->setText(QApplication::translate("ASDInterface", "ToggleLife", 0));

			//Name->setText(QApplication::translate("ASDInterface", "ToggleLife", 0));
			
			Start->setText(QApplication::translate("ASDInterface", "Start", 0));
			ShutDown->setText(QApplication::translate("ASDInterface", "Shut_Down", 0));
		}
};

namespace Ui{
	class ASDInterface: public Ui_ASDInterface {};
}

QT_END_NAMESPACE

#endif
