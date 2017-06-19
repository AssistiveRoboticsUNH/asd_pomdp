/*
pomdp.cpp
Madison Clark-Turner
1/24/2017
*/
#include "../include/asdpomdp/pomdp.h"

POMDP::POMDP(const std::string filename){
	std::cout << "opened: " << filename << std::endl;
	std::ifstream ifile(filename.c_str());
	if(!ifile.good()){
		std::cout << "could not find: " << filename << std::endl;
		return;
	}
	
	std::string s, tag, junk, s_str, a_str, sp_str, o_str;
	int num;
	double prob;

	bool t_built = false;
	bool o_built = false;
	bool r_built = false;
	
	while(!ifile.eof()){
		std::getline(ifile, s);
		std::stringstream ss(s);
		if(s.size() > 0 && s[0] != '#'){
			ss >> tag;
			ss >> junk;

			if(tag == "discount"){
				ss >> discount;
			}
			else if(tag == "states"){
				while(!ss.eof()){
					ss >> junk;
					states.push_back(junk);
				}
			}
			else if(tag == "actions"){
				while(!ss.eof()){
					ss >> junk;
					actions.push_back(junk);
				}
			}
			else if(tag == "observations"){
				ss >> observations;
			}
			else if(tag == "start"){
				while(!ss.eof()){
					ss >> junk;
					start.push_back(junk);
				}
			}
			else if(tag == "T"){
				if(!t_built){
					init3DArray(transitionFunc, states.size(), actions.size(), states.size());
					t_built = true;
				}

				ss >> a_str;
				ss >> junk;
				ss >> s_str;
				ss >> junk;
				ss >> sp_str;
				ss >> prob;

				std::vector<int> s_vec;
				std::vector<int> a_vec;
				std::vector<int> sp_vec;

				if(s_str == "*"){
					for(int s = 0; s < states.size(); s++){
						s_vec.push_back(s);
					}
				}else{
					s_vec.push_back(std::find(states.begin(), states.end(), s_str) - states.begin());
				}
				
				if(a_str == "*"){
					for(int a = 0; a < actions.size(); a++){
						a_vec.push_back(a);
					}
				}else{
					a_vec.push_back(std::find(actions.begin(), actions.end(), a_str) - actions.begin());
				}

				if(sp_str == "*"){
					for(int s = 0; s < states.size(); s++){
						sp_vec.push_back(s);
					}
				}else{
					sp_vec.push_back(std::find(states.begin(), states.end(), sp_str) - states.begin());
				}

				fill3DArray(transitionFunc, s_vec, a_vec, sp_vec, prob);

			}
			else if(tag == "O"){
				if(!o_built){
					init3DArray(observationFunc, states.size(), actions.size(), observations);
					o_built = true;
				}

				ss >> a_str;
				ss >> junk;
				ss >> s_str;
				ss >> junk;
				ss >> o_str;
				ss >> prob;

				std::vector<int> s_vec;
				std::vector<int> a_vec;
				std::vector<int> o_vec;

				if(s_str == "*"){
					for(int s = 0; s < states.size(); s++){
						s_vec.push_back(s);
					}
				}else{
					s_vec.push_back(std::find(states.begin(), states.end(), s_str) - states.begin());
				}
				
				if(a_str == "*"){
					for(int a = 0; a < actions.size(); a++){
						a_vec.push_back(a);
					}
				}else{
					a_vec.push_back(std::find(actions.begin(), actions.end(), a_str) - actions.begin());
				}

				if(o_str == "*"){
					for(int o = 0; o < observations; o++){
						o_vec.push_back(o);
					}
					prob = 1.0/observations;
				}else{
					std::stringstream convert(o_str);
					convert >> num;
					o_vec.push_back(num);
				}

				fill3DArray(observationFunc, s_vec, a_vec, o_vec, prob);
			}
			else if(tag == "R"){
				if(!r_built){
					init3DArray(rewardFunc, actions.size(), states.size(), states.size());
					r_built = true;
				}

				ss >> a_str;
				ss >> junk;
				ss >> s_str;
				ss >> junk;
				ss >> sp_str;
				ss >> junk;
				ss >> junk;
				ss >> junk;
				ss >> prob;

				std::vector<int> s_vec;
				std::vector<int> a_vec;
				std::vector<int> sp_vec;
				
				if(s_str == "*"){
					for(int s = 0; s < states.size(); s++){
						s_vec.push_back(s);
					}
				}else{
					s_vec.push_back(std::find(states.begin(), states.end(), s_str) - states.begin());
				}
				
				if(a_str == "*"){
					for(int a = 0; a < actions.size(); a++){
						a_vec.push_back(a);
					}
				}else{
					a_vec.push_back(std::find(actions.begin(), actions.end(), a_str) - actions.begin());
				}

				if(sp_str == "*"){
					for(int s = 0; s < states.size(); s++){
						sp_vec.push_back(s);
					}
				}else{
					sp_vec.push_back(std::find(states.begin(), states.end(), sp_str) - states.begin());
				}

				fill3DArray(rewardFunc, a_vec, s_vec, sp_vec, prob);
			}
		}
	}
	
	ifile.close();
}

void POMDP::init3DArray(std::vector< std::vector< std::vector< double > > >& array, int _x, int _y, int _z){
	array.clear();
    for(size_t x = 0; x < _x; x++){
        array.push_back( std::vector< std::vector< double > >() );
        for(size_t y = 0; y < _y; y++){
            array[x].push_back(std::vector< double >());
            for(size_t z = 0; z < _z; z++){
                array[x][y].push_back(-1);
            }
        }
    } 
}

void POMDP::fill3DArray(std::vector< std::vector< std::vector< double > > >& array, std::vector<int> _x,
		std::vector<int> _y, std::vector<int> _z, double value){

	for(std::vector<int>::iterator it_x = _x.begin(); it_x != _x.end(); it_x++){
		for(std::vector<int>::iterator it_y = _y.begin(); it_y != _y.end(); it_y++){
			for(std::vector<int>::iterator it_z = _z.begin(); it_z != _z.end(); it_z++){
				array[*it_x][*it_y][*it_z] = value;
			}
		}
	}
}

void POMDP::print(){
	//discount
	std::cout << "discount: " << discount << std::endl;
	//states
	std::cout << "states: ";
	for(int i = 0; i < states.size(); i++){
		std::cout << states[i] << ' ';
	}
	std::cout << std::endl;
	//actions
	std::cout << "actions: ";
	for(int i = 0; i < actions.size(); i++){
		std::cout << actions[i] << ' ';
	}
	std::cout << std::endl;
	//observations
	std::cout << "observations: " << observations << std::endl;
	//start
	std::cout << "start: ";
	for(int i = 0; i < start.size(); i++){
		std::cout << start[i] << ' ';
	}
	std::cout << std::endl;

	//T
	std::cout << "T: " << std::endl;
	for(int s = 0; s < states.size(); s++){
		for(int a = 0; a < actions.size(); a++){
			for(int sp = 0; sp < states.size(); sp++){
				std::cout << s << ' ' << a << ' ' << sp << " : " << transitionFunc[s][a][sp] << std::endl;
			}
		}
	}
	std::cout << std::endl;

	//O
	std::cout << "O: " << std::endl;
	for(int s = 0; s < states.size(); s++){
		for(int a = 0; a < actions.size(); a++){
			for(int o = 0; o < observations; o++){
				std::cout << s << ' ' << a << ' ' << o << " : " << observationFunc[s][a][o] << std::endl;
			}
		}
	}
	std::cout << std::endl;

	//R
	std::cout << "R: " << std::endl;
	for(int a = 0; a < actions.size(); a++){
		for(int s = 0; s < states.size(); s++){
			for(int sp = 0; sp < states.size(); sp++){
				std::cout << a << ' ' << s << ' ' << sp << " : " << rewardFunc[a][s][sp] << std::endl;
			}
		}
	}
	std::cout << std::endl;
}
/*
int main(){
	POMDP* pomdp = new POMDP("asd.pomdp");
	pomdp->print();
	delete pomdp;
}
*/
