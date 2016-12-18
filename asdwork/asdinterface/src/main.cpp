#include "../include/asdinterface/asdinterface.hpp"
#include <QApplication>
#include <QtGui>

int main(int argc, char ** argv){
	ros::init(argc, argv, "asd_interface");
	QApplication a(argc, argv);
	ASDInterface w;
	w.show();
	
	return a.exec();
}
