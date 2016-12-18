#include "../include/node_controllers/audio_recorder.hpp"

AudioLogger::AudioLogger(ros::NodeHandle node): n(node){
	client_record = n.serviceClient<nao_interaction_msgs::AudioRecorder>("/nao_audio/record");
	sub_control = n.subscribe("/control_msgs", 100, &AudioLogger::cb, this);
	sub_recording = n.subscribe("/nao_recording", 100, &AudioLogger::rec, this);
}

AudioLogger::~AudioLogger(){
	ros::shutdown();
}

void AudioLogger::cb(const custom_msgs::control_states States){
	states = States;
}

std::string AudioLogger::getTimeStamp(){
	time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer,80,"%Y-%m-%d-%H-%M-%S", timeinfo);
        std::string str(buffer);
        return str;
}

void AudioLogger::rec(const std_msgs::Bool is_recording){
	if(is_recording.data){
		ROS_INFO("AUDIO RECORDING INITIALIZED");
		//std::string timestamp = states.timestamp;

		std::string timestamp = getTimeStamp();
		std::string file_name = "recording_" + timestamp + ".wav";//bag_path_ + timestamp + ".wav";
		std::cout << file_name << std::endl;

		record.request.front_channel.data = true;
		record.request.left_channel.data = true;
		record.request.right_channel.data = true;
		record.request.rear_channel.data = true;
		record.request.samplerate.data = 16000;
		record.request.audio_type.data = 0;
		record.request.secs.data = 30;
		record.request.file_path.data = file_name;
		ROS_INFO("RECORDING AUDIO");
		client_record.call(record);
	}
	/*else{
		ROS_INFO("SHUTTING DOWN AUDIO RECORDER");
			ros::shutdown();
	}*/
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "nao_record_microphone");
	ros::NodeHandle n;

	std::string bag_path_;
	n.param("bag_path", bag_path_, std::string("~/bag"));


	AudioLogger al(n);

	ros::spin();
	
	return 0;
}
