/*
pomdp.h
Madison Clark-Turner
1/24/2017
*/
#ifndef POMDP_H
#define POMDP_H

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

class POMDP{
public:
	std::vector<std::string> states;
	std::vector<std::string> actions;
	int observations;
	double discount;
	std::vector<std::string> start;

	std::vector< std::vector< std::vector< double > > > transitionFunc;//s, a ,s'
	std::vector< std::vector< std::vector< double > > > observationFunc;//s, a, o
	std::vector< std::vector< std::vector< double > > > rewardFunc;//a, s, s'

	POMDP(const std::string );
	void init3DArray(std::vector< std::vector< std::vector< double > > >&, int, int, int);
	void fill3DArray(std::vector< std::vector< std::vector< double > > >& , std::vector<int>,
			std::vector<int>, std::vector<int>, double );
	void print();
}; 

#endif
