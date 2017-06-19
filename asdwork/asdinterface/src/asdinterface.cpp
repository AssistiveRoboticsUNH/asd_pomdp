/*
asdinterface.cpp
Madison Clark-Turner
1/24/2017
*/
#include "../include/asdinterface/asdinterface.hpp"
#include "../include/asdinterface/ui_asdinterface.hpp"
#include <string.h>

/* Class Constructor: Initializes all of the QT slots and widgets, and initializes all of the subscribers,
 * publishers, and services */
ASDInterface::ASDInterface(QWidget *parent) : QWidget(parent), ui(new Ui::ASDInterface){
	
	/* Sets up UI */
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(on_MyClock_overflow()));
	Mytimer.start(100, this);
	timer->start(100);
	ui->setupUi(this);

	/* Sets up ROS */
	ros::start();
	count = 0; //sets count to 0 so program can go through ros::spinOnce 10 times to solve issue with seg fault
	pub_custom = n.advertise<custom_msgs::control_states>("/control_msgs", 100); // advertises state status
	pub_move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100); // publisher to set nao joint angles
	pub_pose = n.advertise<naoqi_bridge_msgs::BodyPoseActionGoal>("/body_pose/goal", 100); // publisher to make nao switch poses
	pub_run = n.advertise<std_msgs::Bool>("/asdpomdp/run_asd_auto", 100); // publisher to start automated behvioral intervention
	pub_speak = n.advertise<std_msgs::String>("/speech", 100); // publisher to make nao talk
	
	client_stiff = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable", 100); //service client to stiffen nao
	client_record_start = n.serviceClient<std_srvs::Empty>("/data_logger/start"); // service client to start rosbag
	client_record_stop = n.serviceClient<std_srvs::Empty>("/data_logger/stop"); // service client to stop rosbag
	client_wakeup = n.serviceClient<std_srvs::Empty>("/wakeup"); // service client to place nao in rest-mode
	client_rest = n.serviceClient<std_srvs::Empty>("/rest"); // service client to place nao in rest-mode
	life_enable = n.serviceClient<std_srvs::Empty>("/life/enable"); // service client to enable autonomous life
	life_disable = n.serviceClient<std_srvs::Empty>("/life/disable"); // service client to disable autonomous life

	sub_cam = n.subscribe<sensor_msgs::Image>("camera", 1, &ASDInterface::imageCallback, this); //subcriber to get image it.subscribeCamera ASDInterface::proceessGaze
	sub_custom = n.subscribe("control_msgs", 100, &ASDInterface::controlCallback, this); // subscriber to get state status
	sub_nextAct = n.subscribe("/asdpomdp/next_action", 100, &ASDInterface::actionCallback, this);
}

/* Destructor: Frees space in memory where ui was allocated */
ASDInterface::~ASDInterface(){
	delete ui;
}

/* When clock is overflowed, update time */
void ASDInterface::on_MyClock_overflow(){
	QTime time = QTime::currentTime();
	QString text = time.toString("hh:mm:ss");
	ui->MyClock->display(text);
}

/* Updates the displayed Image on UI */
void ASDInterface::UpdateImage(){
	ros::spinOnce(); //spins to call callback to update image information
}

/* Paints the camera image and clock to the UI */
void ASDInterface::paintEvent(QPaintEvent *event){
	QPainter myPainter(this);
	QPointF p(20, 250);
	QPointF p1(435, 170);
	myPainter.drawText(p1, MyClockTimetext); // Draws clock to ui
	if(count >= 10){ // first few frames are corrupted, so cannot draw image until it gets atleast 10 frames of the video stream
		myPainter.drawImage(p, NaoImg); //draws camera image on screen
	}
}

/* Angles the nao's head so that it looks at the participant */
void ASDInterface::centerGaze(){
	nao_msgs::JointAnglesWithSpeed head_angle;
	head_angle.joint_names.push_back("HeadPitch");
	head_angle.joint_angles.push_back(0.1);
	head_angle.speed = 1.0;
	pub_move.publish(head_angle);
}

/* Gets the current timestamp whenever it is called */
std::string ASDInterface::getTimeStamp(){
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time (&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
	std::string str(buffer);
	return str;
}

/* Activates an action if it is called via rostopic */
void ASDInterface::actionCallback(const std_msgs::Int8& msg){
	int act = msg.data;
	centerGaze();
	if(act == 0){
		on_Command_clicked();
	}
	else if(act == 1){
		on_Prompt_clicked();
	}
	else if(act == 2){
		on_Respond_clicked();
	}
	else if(act == 3){
		on_Bye_clicked();
	}
}

/* Makes the nao stand, look at the participant, and enables the nao to wave */
void ASDInterface::on_Start_clicked(){
	
	// stiffen nao and disable autonomous life
	naoqi_bridge_msgs::BodyPoseActionGoal pose;
	std_srvs::Empty stiff;
	life_disable.call(stiff);
	life_on = false;
	client_stiff.call(stiff);
	
	// make nao stand and look at participant
	pose.goal.pose_name = "Stand";
	pub_pose.publish(pose);
	loopRate(40);
	centerGaze();

	// publish state data
	controlstate.startrecord = true;
	ros::Duration(0.9).sleep();
	controlstate.timestamp = getTimeStamp();
	pub_custom.publish(controlstate);
}

void ASDInterface::on_ToggleLife_clicked(){
	// toggles whether autonomous life is enabled or disabled
	std_srvs::Empty stiff;
	if(life_on){
		life_disable.call(stiff);
	}
	else{
		life_enable.call(stiff);
	}

	life_on = !life_on;
}

void ASDInterface::on_Stand_clicked(){
	// make the nao stand
	/*
	naoqi_bridge_msgs::BodyPoseActionGoal pose;
	pose.goal.pose_name = "Stand";
	pub_pose.publish(pose);*/
	std_srvs::Empty stiff;
	client_wakeup.call(stiff);
}

void ASDInterface::on_Rest_clicked(){
	// make the nao rest
	std_srvs::Empty stiff;
	client_rest.call(stiff);
}

void ASDInterface::on_AngleHead_clicked(){
	// force the nao to angle its head towards the participant
	centerGaze();
}

void ASDInterface::on_Run_clicked(){
	// start an automated therapy session
	std_msgs::Bool msg;
	msg.data = 1;
	pub_run.publish(msg);
}

void ASDInterface::on_StartRecord_clicked(){
	std_msgs::Bool record;
	std_srvs::Empty stiff;
	client_record_start.call(stiff);
	recording = true;
}

void ASDInterface::on_StopRecord_clicked(){
	std_msgs::Bool record;
	std_srvs::Empty stop;
	if(recording)
		client_record_stop.call(stop);
	recording = false;
}

/* Commands patient to say Hello and wave */
void ASDInterface::on_Command_clicked(){
	std_msgs::String words;
	name = ui->Name->toPlainText().toStdString();
	words.data = "\\RSPD=70\\Hello, "+name+"!";

	controlstate.startwave2 = true;

	//wave and say hello
	pub_custom.publish(controlstate);
	loopRate(15);
	pub_speak.publish(words);
}

/* Prompts patient, says hello, and waves  */
void ASDInterface::on_Prompt_clicked(){
	std_msgs::String words;
	name = ui->Name->toPlainText().toStdString();
	words.data = "\\RSPD=70\\"+name+", say hello!";

	controlstate.startwave2 = true;

	//wave and say hello
	pub_custom.publish(controlstate);
	loopRate(15);
	pub_speak.publish(words);
}

/* Rewards patient  */
void ASDInterface::on_Respond_clicked(){
	std_msgs::String words;
	words.data = "\\RSPD=70\\Great Job!";

	//say great job!
	pub_speak.publish(words);
}

/* Makes nao say goodbye */
void ASDInterface::on_Bye_clicked(){
	std_msgs::String words;
	name = ui->Name->toPlainText().toStdString();
	words.data = "\\RSPD=70\\Goodbye, "+name+".";

	//say good bye
	pub_speak.publish(words); 
}

/* Shuts down ROS and program */
void ASDInterface::on_ShutDown_clicked(){
	// shutsdown this node with ros and exits
	std_srvs::Empty stop;
	if(recording)
		client_record_stop.call(stop);	

	// publish shutdown to controlstate to get other nodes to terminate
	controlstate.shutdown = true;
	pub_custom.publish(controlstate);

	ros::shutdown();
	exit(0);
}

/* Updates image data and gui */
void ASDInterface::timerEvent(QTimerEvent*) {
	UpdateImage();
	update();
}

/* Call back to store image data from camera using ROS and converts it to QImage */
void ASDInterface::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	QImage myImage(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
	NaoImg = myImage.rgbSwapped();
	count++;
}

/* Loop rate to make NAO wait for i amount of seconds */
void ASDInterface::loopRate(int loop_times){
	ros::Rate loop_rate(15);
	for(int i = 0; i < loop_times; i++){
		loop_rate.sleep();
	}
}

/* Custom Msg Callback */
void ASDInterface::controlCallback(const custom_msgs::control_states States){
	centerGaze();
	controlstate = States;
	
	if(!controlstate.startwave1 && !controlstate.startwave2){
		ros::Duration(2).sleep();
		centerGaze();
	}
}