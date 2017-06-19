/*
pomdppolicy.cpp
Madison Clark-Turner
1/24/2017
*/
#include "../include/asdpomdp/pomdppolicy.h"

POMDPPolicy::POMDPPolicy(POMDP* pomdp_data, const std::string filename): pomdp(pomdp_data){
	//filename is the list of alpha vectors
	beliefState.push_back(0.5);
	beliefState.push_back(0.5);

	std::ifstream ifile(filename.c_str());
	if(!ifile.good()){
		std::cout << "could not find: " << filename << std::endl;
		return;
	}
	std::string s;

	while(!ifile.eof()){
		std::getline(ifile, s);
		std::stringstream ss(s);
		
		if(s.size() > 0 && s[0] != '#'){
			AlphaVector a;
			double x0, x1;
			ss >> a.action;
			std::getline(ifile, s);
			std::stringstream ss(s);
			ss >> a.yint >> x1;

			a.m = x1 - a.yint;
			alphas.push_back(a);
		}
	}
}

void POMDPPolicy::reset(){
	//reset the POMDP policy
	beliefState.clear();
	beliefState.push_back(0.5);
	beliefState.push_back(0.5);
}

void POMDPPolicy::updateBeliefState(int obs, int act){
	//update the POMDP's belief state
	//belief = O(o|s', a) sum[over s] T(s'|s,a)b(s)
	double total = 0;
	for(int b = 0; b < beliefState.size(); b++){
		double sum = 0;
		for(int s = 0; s < pomdp->states.size(); s++){
			sum += pomdp->transitionFunc[s][act][b] * beliefState[s];
		}
		beliefState[b] = pomdp->observationFunc[b][act][obs] * sum;
		total += beliefState[b];
	}
	for(int b = 0; b < beliefState.size(); b++){
		beliefState[b] /= total;
	}
}

int POMDPPolicy::selectAction(){
	//select the next action to perform based on the current belief state
	double b = beliefState[1];//probability of non-compliance

	AlphaVector max;
	double maxval;
	for(int a  = 0; a < alphas.size(); a++){
		double val = alphas[a].m * b + alphas[a].yint;
		if(a == 0 || val > maxval){
			max = alphas[a];
			maxval = val;
		}
	}
	return max.action;
}

void POMDPPolicy::printBelief(){
	//print the belief state
	for (int b = 0; b < beliefState.size(); b++){
		std::cout << beliefState[b] << ", ";
	}
	std::cout << std::endl;
}
