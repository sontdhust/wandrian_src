
#include "../../../include/plans/boustrophedon_off/environmentoff.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon_off {

Environmentoff::Environmentoff(ObstaclePtr environment, std::list<ObstaclePtr> obstacles) :
		environment(environment), obstacles(obstacles) {
}

Environmentoff::Environmentoff(std::string namefile){
	this->namefile = namefile;
}

ObstaclePtr Environmentoff::get_environment(){
	return this->environment;
}

std::list<ObstaclePtr> Environmentoff::get_obstacles(){
	return this->obstacles;
}
void Environmentoff::set_environment(){
	std::string size;
	std::string position;
	std::string line;

	int i, flag;

	std::fstream myReadFile;

	if(this->namefile.compare("") != 0){

		myReadFile.open(this->namefile.c_str());

		std::cout <<"out"<< this->namefile <<std::endl;

		if(myReadFile.is_open()){
			std::cout <<"Out2"<< this->namefile <<std::endl;
			while (getline(myReadFile,line)!=NULL) {

						std::cout <<"Out2"<< this->namefile <<std::endl;

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
						std::cout << "Position" <<position<<"Size"<<size<<std::endl;

						for (unsigned int var = 0; var <position.length(); ++var) {
								if(position[var] == ','){
									flag = var;
									break;
								}
						}

			//			flag = commaposition(position);
			//			point.setPoint(stringtonumber(position.substr(0,flag)),
			//								stringtonumber(position.substr(flag+1, position.length())));
			//				flag = commaposition(size);
			//
			//				cell.setCenter(point);
			//				cell.setSize(stringtonumber(size.substr(0,flag)),
			//						stringtonumber(size.substr(flag+1, size.length())));
			//
			//				listpoint.push_back(point);
			//				listcell.push_back(cell);
						}
		}else{
			std::cout<<"can't open file "<<std::endl;
		}

		myReadFile.close();
	}
}


}
}
}
