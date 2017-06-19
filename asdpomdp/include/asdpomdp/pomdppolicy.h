/*
pomdppolicy.h
Madison Clark-Turner
1/24/2017
*/
#ifndef POMDPPOLICY_H
#define POMDPPOLICY_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include "pomdp.h"

class POMDPPolicy{
private:
	typedef struct {
		int action;
		double yint;
		double m;
	} AlphaVector;

	POMDP* pomdp;

	std::vector<double> beliefState;
	std::vector<AlphaVector> alphas;

public: 
	POMDPPolicy(POMDP*, const std::string);

	void updateBeliefState(int, int);
	int selectAction();
	void printBelief();
	void reset();
};

#endif
