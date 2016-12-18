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
	pub_speak = n.advertise<std_msgs::String>("/speech", 100); //publisher to make nao talk
	//pub_gaze = n.advertise<std_msgs::String>("/gaze", 100); //publisher to make nao talk
	pub_pose = n.advertise<naoqi_bridge_msgs::BodyPoseActionGoal>("/body_pose/goal", 100); //publisher to make nao switch poses
	client_stiff = n.serviceClient<std_srvs::Empty>("/body_stiffness/enable", 100); //service client to unstiffen nao
	client_record_start = n.serviceClient<std_srvs::Empty>("/data_logger/start"); // service client to start rosbag
	client_record_stop = n.serviceClient<std_srvs::Empty>("/data_logger/stop"); //service client to stop rosbag
	client_rest = n.serviceClient<std_srvs::Empty>("/rest"); 
	life_enable = n.serviceClient<std_srvs::Empty>("/life/enable"); // service client to start rosbag
	life_disable = n.serviceClient<std_srvs::Empty>("/life/disable");
	//sub_cam = n.subscribe<sensor_msgs::Image>("/nao_robot/camera/top/camera/image_raw", 100, &ASDInterface::imageCallback, this); //subcriber to get image
	sub_cam = n.subscribe<sensor_msgs::Image>("attention_tracker/faces/image", 1, &ASDInterface::imageCallback, this); //subcriber to get image it.subscribeCamera ASDInterface::proceessGaze
	pub_move = n.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 100);

	pub_record = n.advertise<std_msgs::Bool>("/nao_recording", 100);

	sub_custom = n.subscribe("control_msgs", 100, &ASDInterface::controlCallback, this); // subscriber to get state status
	pub_custom = n.advertise<custom_msgs::control_states>("/control_msgs", 100); // advertises state status

	pub_actFinished = n.advertise<std_msgs::Int8>("/action_finished", 100);
	sub_nextAct = n.subscribe("next_action", 100, &ASDInterface::actionCallback, this);

	/* Creates file path to put timestamps in txt document */
	std::string file_path;
	//file_path = getenv("asdinterface");
	//file_path += "data/timestamps.txt";
	fout.open("~/asd_data/timestmaps/timestamps.txt", std::ofstream::out | std::ofstream::app);
	std::string timestamp;
	timestamp = getTimeStamp();
	fout << "UI LAUNCHED: " << timestamp << "\n\n";
}

/* Destructor: Frees space in memory where ui was allocated */
ASDInterface::~ASDInterface(){
	delete ui;
}

void ASDInterface::actionCallback(const std_msgs::Int8& msg){
	int act = msg.data;
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

void ASDInterface::centerGaze(){
	nao_msgs::JointAnglesWithSpeed head_angle;
	head_angle.joint_names.push_back("HeadPitch");
	head_angle.joint_angles.push_back(0.1);
	head_angle.speed = 1.0;
	ROS_INFO("Change Head Pitch");
	pub_move.publish(head_angle);
}

/* Makes NAO stand and comment on someone entering room */
void ASDInterface::on_Start_clicked(){
	
	/* Gets current timestamp and prints it to txt file */
	std::string timestamp;
	timestamp = getTimeStamp();
	fout << "\tStart: " << timestamp << "\n";

	/* Gets NAO to stiffen, standup, and start audio and rosbag recording */
	naoqi_bridge_msgs::BodyPoseActionGoal pose;
	std_srvs::Empty stiff;
	life_disable.call(stiff);
	life_on = false;
	client_stiff.call(stiff);
	
	
	
	pose.goal.pose_name = "Stand";
	pub_pose.publish(pose);

	loopRate(40);
	centerGaze();


	controlstate.startrecord = true;
	ros::Duration(0.9).sleep();

	//client_record_start.call(stiff);
	//recording = true;
	timestamp = getTimeStamp();
	controlstate.timestamp = timestamp;
	pub_custom.publish(controlstate);
}

void ASDInterface::on_ToggleLife_clicked(){
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
	naoqi_bridge_msgs::BodyPoseActionGoal pose;
	pose.goal.pose_name = "Stand";
	pub_pose.publish(pose);
}

void ASDInterface::on_Rest_clicked(){
	std_srvs::Empty stiff;
	client_rest.call(stiff);
}

void ASDInterface::on_AngleHead_clicked(){
	centerGaze();
}

void ASDInterface::on_StartRecord_clicked(){
	std_msgs::Bool record;
	std_srvs::Empty stiff;
	client_record_start.call(stiff);
	recording = true;
	record.data = recording;
	pub_record.publish(record);
}

void ASDInterface::on_StopRecord_clicked(){
	std_msgs::Bool record;
	std_srvs::Empty stop;
	if(recording)
		client_record_stop.call(stop);
	recording = false;
	//record.data = recording;
	//pub_record.publish(record);
}

/* Commands patient to say Hello and wave */
void ASDInterface::on_Command_clicked(){
	
	// Gets current timestamp of when button was clicked
	std::string timestamp;
	timestamp = getTimeStamp();

	// writes timestamp of when button was clicked to file
	fout << "\tCommand: " << timestamp << "\n";
	
	// makes robot say hello and pings other node to get robot to wave
	std_msgs::String words;

	words.data = "\\RSPD=70\\Hello, "+name+"!";
	controlstate.startwave2 = true;
	actionFinished.data = 0;
	pub_custom.publish(controlstate);
	loopRate(15);
	pub_speak.publish(words);
}

/* Greets patient, says hello, and waves  */
void ASDInterface::on_Prompt_clicked(){
	
	// Gets current time stamp when button was clicked
	std::string timestamp;
	timestamp = getTimeStamp();

	// writes timestamp of when button was clicked to file
	fout << "\tPrompt: " << timestamp << "\n";
	
	// Robot greets person
	std_msgs::String words;
        words.data = "\\RSPD=70\\"+name+", say hello!";
	controlstate.startwave2 = true;
	actionFinished.data = 1;
        pub_custom.publish(controlstate);
	loopRate(15);
        pub_speak.publish(words);
}

/* Greets patient, says hello, and waves  */
void ASDInterface::on_Respond_clicked(){
	//ROS_INFO("Respond clicked");

	// Gets current time stamp when button was clicked
	std::string timestamp;
	timestamp = getTimeStamp();

	// writes timestamp of when button was clicked to file
	fout << "\tRespond: " << timestamp << "\n";
	
	// Robot greets person
	std_msgs::String words;
        words.data = "\\RSPD=70\\Great Job!";

	actionFinished.data = 2;


    pub_speak.publish(words);
    pub_actFinished.publish(actionFinished);
}

/* Makes NAO crouch and say goodbye */
void ASDInterface::on_Bye_clicked(){
	
	// Gets current time stamp when button was clicked
	std::string timestamp;
	timestamp = getTimeStamp();

	// writes timestamp of when button was clicked to file
	fout << "\tBye: " << timestamp << "\n";

	// Makes robot say good bye and crouch
	std_msgs::String words;
	naoqi_bridge_msgs::BodyPoseActionGoal pose;
	words.data = "\\RSPD=70\\Goodbye, "+name+".";
	//Need to get robot to crouch//
	pub_speak.publish(words); 
	actionFinished.data = 3;
	pub_actFinished.publish(actionFinished);
	//std_srvs::Empty stiff;
	//life_enable.call(stiff);
}

/* Shuts down ROS and program */
void ASDInterface::on_ShutDown_clicked(){
		
	// Gets current timestamp
	std::string timestamp;
	timestamp = getTimeStamp();
	
	// Prints timestamp of when button was clicked to file
	fout << "\nUI Terminated: " << timestamp << "\n\n";
	fout.close();

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
	//Mat2QImage
	QImage myImage(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
	NaoImg = myImage.rgbSwapped();
	count++;
}
/*
QImage Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp; // make the same cv::Mat
     cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
     QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     dest.bits(); // enforce deep copy, see documentation 
     // of QImage::QImage ( const uchar * data, int width, int height, Format format )
     return dest;
}
*/
/* Loop rate to make NAO wait for i amount of seconds */
void ASDInterface::loopRate(int loop_times){
	ros::Rate loop_rate(15);
	for(int i = 0; i < loop_times; i++){
		loop_rate.sleep();
	}
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

/* Custom Msg Callback */
void ASDInterface::controlCallback(const custom_msgs::control_states States){
	controlstate = States;
	
	if(!controlstate.startwave1 && !controlstate.startwave2){
		pub_actFinished.publish(actionFinished);
		//loopRate(40);
		centerGaze();
	}
}
