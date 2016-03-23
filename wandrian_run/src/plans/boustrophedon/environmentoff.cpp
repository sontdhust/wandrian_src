
#include "../../../include/plans/boustrophedon/environmentoff.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon{

Environmentoff::Environmentoff(ObstaclePtr environment, std::list<ObstaclePtr> obstacles) :
		environment(environment), obstacles(obstacles) {
}

Environmentoff::Environmentoff(std::string namefile){
	this->namefile = namefile;
	set_environment();
}

ObstaclePtr Environmentoff::get_environment(){
	return this->environment;
}

std::list<ObstaclePtr> Environmentoff::get_obstacles(){

	return this->obstacles;
}
int Environmentoff::commaposition(std::string str){
	for(unsigned int position=0; position <str.length(); ++position){
		if(str[position]==',')return position;
	}
	return 0;
}
void Environmentoff::set_environment(){
	std::string size;
	std::string position;
	std::string line;

	PointPtr center;

	double sizex, sizey;

	int i, flag;

	std::fstream myReadFile;

	if(this->namefile.compare("") != 0){

		myReadFile.open(this->namefile.c_str());

		std::cout <<"out"<< this->namefile <<std::endl;

		if(myReadFile.is_open()){
			std::cout <<"Out2"<< this->namefile <<std::endl;
				int i=0;
				int flag = 0;
				while (getline(myReadFile,line)!=NULL) {
					i=0;
					flag = 0;
					position = "";
					size = "";
					for(unsigned int var=0;var<line.length(); ++var){
						if(line[var]=='('){
							if (position.compare("") == 0)
								i=1;
							else i = 2;
							flag = var+1;
							continue;
						}
						if(line[var]==')'){
							i = 0;
							continue;
						}
						if(i==1){
							position.insert(var-flag, 1 , line[var]);
						}
						if(i==2){
							size.insert(var-flag, 1 , line[var]);
						}
					}
					flag = 0;

					std::cout <<"Position: "<< position <<std::endl;
					std::cout <<"Size: "<< size <<std::endl;
					for (unsigned int var = 0; var <position.length(); ++var) {
						if(position[var] == ','){
							flag = var;
							break;
						}
					}

					flag = commaposition(position);
					center = PointPtr(new Point( strtod(position.substr(0,flag).c_str(),NULL), strtod(position.substr(flag+1, position.length()).c_str(),NULL)));

					flag = commaposition(size);
					sizex = strtod(size.substr(0,flag).c_str(),NULL);
					sizey = strtod(size.substr(flag+1, size.length()).c_str(),NULL);
					std::cout <<"Position ("<<center->x <<" ,"<< center->y << " )"<<std::endl;
					std::cout <<"Size : x ="<< sizex <<" y = "<< sizey <<std::endl;

					if(!this->environment){
						this->environment = ObstaclePtr(new Obstacle(center, sizex, sizey));
						std::cout <<"ADD en "<<std::endl;
					}else{
						this->obstacles.push_back(ObstaclePtr(new Obstacle(center, sizex, sizey)));
						std::cout <<"ADD obstacles "<<std::endl;
					}
				}
		}else{
			std::cout<<"Can't open file "<<std::endl;
		}

		myReadFile.close();
	}
}


}
}
}
