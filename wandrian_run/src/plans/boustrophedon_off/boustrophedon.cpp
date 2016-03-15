/*
 * boustrophedon_cd.cpp
 *
 */

#include "../../../include/plans/boustrophedon_off/boustrophedon.hpp"

namespace wandrian {
namespace plans {
namespace boustrophedon_off {

Boustrophedon::Boustrophedon() :
    robot_size(0),environment_size(0) {
}

Boustrophedon::~Boustrophedon() {
}

//, double environment_size

void Boustrophedon::initialize(PointPtr starting_point, double robot_size) {
  this->robot_size = robot_size;
  //this->environment_size = environment_size;

  path.insert(path.end(), starting_point);
}

void Boustrophedon::cover() {
 // old_cells.insert(starting_cell);
  boustrophedon_cd();
}

void Boustrophedon::set_behavior_see_obstacle(
    boost::function<bool(VectorPtr, double)> behavior_see_obstacle) {
  this->behavior_see_obstacle = behavior_see_obstacle;
}

bool Boustrophedon::go_to(PointPtr position, bool flexibly) {
  std::cout << "    pos: " << position->x << "," << position->y << "\n";
  path.insert(path.end(), position);

  if (behavior_go_to)
    return behavior_go_to(position, flexibly);
  return true;
}


bool Boustrophedon::go_go(SpacePtr space){
    double x = (space->get_center()->x*2 - space->get_sizex() + robot_size)/2 ;
    double y = (space->get_center()->y*2 - space->get_sizey() + robot_size)/2 ;

    PointPtr starting_point = PointPtr(new Point(x,y));
    go_to(starting_point, STRICTLY);

    std::cout << "\033[1;34mLast_position-\033[0m\033[1;31m\033[0m: "
    << starting_point->x << "," << starting_point->y << "\n";
    std::cout << "\033[1;34mddd_position-\033[0m\033[1;31m\033[0m: "
       << space->get_sizex() << "," << space->get_sizey() << "\n";
    

    PointPtr last_position;
    PointPtr new_position;

    int flag = 0;

    for (int i = 0; i < space->get_sizex(); ++i){
      if (i!=0){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x+0.5, last_position->y));
        go_to(new_position, STRICTLY);
      }
      std::cout << "\033[1;34mTest_position-\033[0m\033[1;31m\033[0m: "
            << i << "\n";
      for (int j = 0; j < space->get_sizey()*2-1; ++j){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x, last_position->y + 0.5));
        go_to(new_position, STRICTLY);
      }
      last_position = *(--path.end());
      new_position = PointPtr(new Point(last_position->x+0.5, last_position->y));
      go_to(new_position, STRICTLY);
      
      for (int j = 0; j < space->get_sizey()*2-1; ++j){
        last_position = *(--path.end());
        new_position = PointPtr(new Point(last_position->x, last_position->y - 0.5));
        go_to(new_position, STRICTLY);
      }
    }
      return true;
}


bool Boustrophedon::go_with(VectorPtr orientation, double step) {
  PointPtr last_position = *(--path.end());
  PointPtr new_position = PointPtr(
      new Point(*last_position + *orientation * step * robot_size / 2));
  return go_to(new_position, STRICTLY);
}


void Boustrophedon::dfs(SpacePtr space){
	std::list<SpacePtr>::iterator inspectLC;
	space->status_visited = true;
	double x, y;
	std::cout <<"Visit Space"<< space->get_sizex()<< std::endl;
	go_go(space);

	for (inspectLC = space->children.begin(); inspectLC != space->children.end(); ++inspectLC) {
		if((*inspectLC)->status_visited == false){
			//TODO: Space size odd
		     x = (space->get_center()->x*2 + space->get_sizex() - robot_size)/2 ;
		     y = ((*inspectLC)->get_center()->y*2 - (*inspectLC)->get_sizey() + robot_size)/2 ;

		     (*inspectLC)->set_point_backtrack(PointPtr(new Point(x,y)));
			go_to((*inspectLC)->point_backtrack, STRICTLY);
			dfs(*inspectLC);
			go_to((*inspectLC)->point_backtrack, STRICTLY);

		}

	}
}

std::list<SpacePtr> Boustrophedon::create_space(ObstaclePtr environment, std::list<VerticesPtr> list_vertices){

	std::list<VerticesPtr> listvertices_temp;
	std::list<VerticesPtr>::iterator inspectLV;
	std::list<VerticesPtr>::iterator inspectLVT;

	std::list<SpacePtr> list_space;
	std::list<SpacePtr>::iterator listLS;
	PointPtr center_temp;
	int i =1, j=1;
	double sizex = 0, sizey = 0;

	VerticesPtr vertices_previous;

		//Create list space!
	for(inspectLV = list_vertices.begin(), j=1, i= 1; j <= list_vertices.size(); ++inspectLV, ++j){
		std::cout <<"V"<< j <<"("<<(*inspectLV)->get_positions()->x <<", "<<(*inspectLV)->get_positions()->y<<" )"<< std::endl;

		listvertices_temp.sort(Vertices::compare_positionsy);
		if(listvertices_temp.empty()&&(inspectLV == list_vertices.begin())){
			std::cout<<"Empty1"<<std::endl;
			listvertices_temp.push_back(*inspectLV);
			++inspectLV;
			++j;
			listvertices_temp.push_back(*inspectLV);
			continue;
		}

		std::cout<<"List temp current "<<std::endl;
		for (inspectLVT = listvertices_temp.begin();  inspectLVT != listvertices_temp.end() ; ++inspectLVT){
			std::cout <<"V"<<"("<<(*inspectLVT)->get_positions()->x <<", "<<(*inspectLVT)->get_positions()->y<<" )"<< std::endl;
		}

		//1. Create one space
			// + Center
			// + Size
		//2. Push + pop: Temp

		if((*inspectLV)->left_compared_center()||((*inspectLV)->get_polygon() == environment)){
			for (inspectLVT = listvertices_temp.begin();  inspectLVT != listvertices_temp.end() ; ++inspectLVT) {
				if(inspectLVT == listvertices_temp.begin()){
					vertices_previous = *inspectLVT;
					continue;
				}
				if((*inspectLVT)->get_positions()->y < (*inspectLV)->get_positions()->y){
					vertices_previous = *inspectLVT;
					continue;
				}

				// Create space
				sizex =(*inspectLV)->get_positions()->x - (*inspectLVT)->get_positions()->x;
				sizey =(*inspectLVT)->get_positions()->y - vertices_previous->get_positions()->y;

				center_temp = PointPtr(new Point((*inspectLVT)->get_positions()->x + sizex/2,
							(*inspectLVT)->get_positions()->y - sizey/2));

				list_space.push_back(SpacePtr(new Space(center_temp, sizex, sizey)));

				//Remove : two vetices space left
				listvertices_temp.remove(vertices_previous);
				listvertices_temp.remove(*inspectLVT);

				//Push: two vertices space right
				listvertices_temp.push_back(VerticesPtr(new Vertices(PointPtr(new Point(center_temp->x + sizex/2, center_temp->y + sizey/2 )),
																		 ObstaclePtr(new Obstacle(center_temp, sizex, sizey)))));

				if((*inspectLV)->get_positions()->y != environment->get_center()->y - environment->get_sizey()/2){
					listvertices_temp.push_back(VerticesPtr(new Vertices(PointPtr(new Point(center_temp->x + sizex/2, center_temp->y - sizey/2 )),
																					 ObstaclePtr(new Obstacle(center_temp, sizex, sizey)))));
					listvertices_temp.push_back(*inspectLV);
				}
					++inspectLV;
					++j;
					listvertices_temp.push_back(*inspectLV);
					break;
				}
			}
			else{
				if(((*inspectLV)->get_positions()->y == environment->get_center()->y - environment->get_sizey()/2 )||
				  ((*inspectLV)->get_positions()->y == environment->get_center()->y + environment->get_sizey()/2)) {
					listvertices_temp.push_back(*inspectLV);
					++inspectLV;
					++j;
					continue;
				}

				for (inspectLVT = listvertices_temp.begin();  inspectLVT != listvertices_temp.end() ; ++inspectLVT) {
					std::cout <<"V current "<<(*inspectLV)->get_positions()->y <<std::endl;
					std::cout <<"V temp "<<(*inspectLVT)->get_positions()->y <<std::endl;
					if((*inspectLV)->get_positions()->y == (*inspectLVT)->get_positions()->y){
						std::cout <<(*inspectLV)->get_positions()->y <<std::endl;
						break;
					}
					vertices_previous = *inspectLVT;
				}

				std::cout<<"Print2else "<<std::endl;

				if((*inspectLV)->upon_compared_center()){
				  vertices_previous = *inspectLVT;
				  ++inspectLVT;
				}

				sizex = (*inspectLV)->get_positions()->x - vertices_previous->get_positions()->x;
				sizey = (*inspectLVT)->get_positions()->y - vertices_previous->get_positions()->y;

				center_temp = PointPtr(new Point(vertices_previous->get_positions()->x + sizex/2,vertices_previous->get_positions()->y +sizey/2));

				list_space.push_back(SpacePtr(new Space(center_temp, sizex, sizey)));

				//Remove : two vetices space left
				listvertices_temp.remove(vertices_previous);
				listvertices_temp.remove(*inspectLVT);

				if((*inspectLV)->upon_compared_center()){
					listvertices_temp.push_back(VerticesPtr(new Vertices(PointPtr(new Point(center_temp->x + sizex/2, center_temp->y + sizey/2 )),
																		 ObstaclePtr(new Obstacle(center_temp, sizex, sizey)))));
					std::cout<<center_temp->x + sizex/2<< center_temp->y + sizey/2 <<std::endl;
				}
				else{
					listvertices_temp.push_back(VerticesPtr(new Vertices(PointPtr(new Point(center_temp->x + sizex/2, center_temp->y - sizey/2 )),
																		 ObstaclePtr(new Obstacle(center_temp, sizex, sizey)))));
				}

				if((*inspectLV)->get_positions()->x == environment->get_center()->x + environment->get_sizex()/2){
					++inspectLV;
					++j;
				}
			}
		}

//create prant and child?
	return list_space;

}

std::list<VerticesPtr> Boustrophedon::create_vertices(ObstaclePtr environment, std::list<ObstaclePtr> listobstacle){
	std::list<PointPtr> list_point;
	std::list<PointPtr>::iterator inspectLP;

	std::list<VerticesPtr> list_vertices;
	std::list<VerticesPtr>::iterator inspectLV;
	std::list<ObstaclePtr>::iterator inspectLO;

	int i =1, j =1;

	std::cout <<"Environment : 0(" <<environment->get_center()->x<<" ,"<< environment->get_center()->y <<")"<<
							" Size:"<<"("<<environment->get_sizex()<<" ,"<<environment->get_sizey()<<")"<<std::endl;

	// Create list vertices
	for(inspectLO = listobstacle.begin(), i = 1; inspectLO!= listobstacle.end(); ++inspectLO){
		std::cout <<"Obstacle "<< i++ <<":O("<<(*inspectLO)->get_center()->x<<" , "<<
							(*inspectLO)->get_center()->y<<")"<<std::endl;
		list_point = (*inspectLO)->get_bound();

		for ( inspectLP = list_point.begin(), j=1;  inspectLP != list_point.end(); ++inspectLP) {
			std::cout <<"V"<< j++ <<"("<<(*inspectLP)->x <<", "<<(*inspectLP)->y <<" )"<<std::endl;
			list_vertices.push_back(VerticesPtr(new Vertices(*inspectLP, *inspectLO)));
		}
		list_point.clear();
	}

	//Add vertices enviroment

	list_point = environment->get_bound();
	for(inspectLP = list_point.begin(); inspectLP != list_point.end(); ++inspectLP){
		std::cout <<"What the hell"<< std::endl;

		std::cout << "O"<<(*inspectLP)->x << (*inspectLP)->y << std::endl;

		for(inspectLV = list_vertices.begin(),j=0; inspectLV != list_vertices.end(); ++inspectLV){

			std::cout <<"V"<<(*inspectLV)->get_positions()->x<<" "<<(*inspectLV)->get_positions()->y << std::endl;

			if(((*inspectLV)->get_positions()->x == (*inspectLP)->x)&&((*inspectLV)->get_positions()->y == (*inspectLP)->y)){
				j = 1;
				list_vertices.remove(*inspectLV);
				std::cout <<"Sao the nhi"<< std::endl;
				break;
			}

		}

		if(j!= 1){
			list_vertices.push_back(VerticesPtr(new Vertices(*inspectLP, environment)));
		}
	}

	list_vertices.sort(Vertices::compare_positionsx);
	std::cout <<" " <<std::endl;

	return list_vertices;
}

void Boustrophedon::boustrophedon_cd() {

	std::list<ObstaclePtr> listobstacle;
	std::list<ObstaclePtr>::iterator inspectLO;
	ObstaclePtr environment;



	std::list<VerticesPtr> list_vertices;
	std::list<VerticesPtr>::iterator inspectLV;

	std::list<SpacePtr> list_space;
	std::list<SpacePtr>::iterator listLS;


	environment = ObstaclePtr(new Obstacle(PointPtr(new Point(0,0)), 6 ,6 ));

	listobstacle.push_back(ObstaclePtr(new Obstacle(PointPtr(new Point(2,-2)) ,2 ,2 )));
	listobstacle.push_back(ObstaclePtr(new Obstacle(PointPtr(new Point(-1.5, 0)) ,1 ,2 )));
	listobstacle.push_back(ObstaclePtr(new Obstacle(PointPtr(new Point(-0.75,-2.5)) , 1.5 ,1 )));


	//Create vertices
	list_vertices = create_vertices(environment, listobstacle);


	list_space = create_space(environment, list_vertices);


	for (listLS = list_space.begin(); listLS != list_space.end(); ++listLS) {
		std::cout <<"O("<<(*listLS)->get_center()->x <<" ,"<< (*listLS)->get_center()->y <<" ) ";
		std::cout <<"Size("<<(*listLS)->get_sizex() <<" ,"<< (*listLS)->get_sizey() <<" )"<< std::endl;
	}



//	SpacePtr s1 = SpacePtr(new Space(PointPtr(new Point(-2.5,0)), 1 ,6 ));
//	SpacePtr s2 = SpacePtr(new Space(PointPtr(new Point(-1.5,2)), 1 ,2 ));
//	SpacePtr s3 = SpacePtr(new Space(PointPtr(new Point(-1.5,-2)), 1 ,2 ));
//	SpacePtr s4 = SpacePtr(new Space(PointPtr(new Point(0,0)), 2 ,6 ));
//	SpacePtr s5 = SpacePtr(new Space(PointPtr(new Point(2,1)), 2 ,4 ));
//
//	listpoint.push_back(s1);
//	s1->children.push_back(s3);
//	s1->children.push_back(s2);
//	listpoint.push_back(s2);
//	s2->set_parent(s1);
//	listpoint.push_back(s3);
//	s3->set_parent(s1);
//	s3->children.push_back(s4);
//	listpoint.push_back(s4);
//	s4->set_parent(s3);
//	s4->children.push_back(s5);
//	listpoint.push_back(s5);
//	s5->set_parent(s4);
//
//	for(inspectLP = listpoint.begin(), i = 1; inspectLP != listpoint.end(); ++inspectLP){
//		std::cout <<"Space:"<< i++ <<": "<<std::endl;
//		if((*inspectLP)->status_visited == false){
//			dfs(*inspectLP);
//		}
//	}

}
}
}
}
