 for (inspectLVT = listvertices_temp.begin();
        inspectLVT != listvertices_temp.end(); ++inspectLVT) {
      std::cout << "V" << "(" << (*inspectLVT)->get_position()->x << ", "
          << (*inspectLVT)->get_position()->y << " )" << std::endl;
    }

    // 1. Create one space
    //   + Center
    //   + Size
    // 2. Push + pop: Temp

    if ((*inspectLV)->left_compared_center()
        || ((*inspectLV)->get_polygon() == environment)) {
      for (inspectLVT = listvertices_temp.begin();
          inspectLVT != listvertices_temp.end(); ++inspectLVT) {
        if (inspectLVT == listvertices_temp.begin()) {
          vertices_previous = *inspectLVT;
          continue;
        }
        if ((*inspectLVT)->get_position()->y
            < (*inspectLV)->get_position()->y) {
          vertices_previous = *inspectLVT;
          continue;
        }

        // Create space
        size_x = (*inspectLV)->get_position()->x
            - (*inspectLVT)->get_position()->x;
        size_y = (*inspectLVT)->get_position()->y
            - vertices_previous->get_position()->y;

        center_temp = PointPtr(
            new Point((*inspectLVT)->get_position()->x + size_x / 2,
                (*inspectLVT)->get_position()->y - size_y / 2));

        list_space.push_back(SpacePtr(new Space(center_temp, size_x, size_y)));

        // Remove : two vetices space left
        listvertices_temp.remove(vertices_previous);
        listvertices_temp.remove(*inspectLVT);

        // Push: two vertices space right



        if ((*inspectLV)->get_position()->y
            != environment->get_center()->y + environment->get_height() / 2) {
          listvertices_temp.push_back(
              VerticesPtr(
                  new Vertices(
                      PointPtr(
                          new Point(center_temp->x + size_x / 2,
                              center_temp->y + size_y / 2)),
                      RectanglePtr(
                          new Rectangle(center_temp, size_x, size_y)))));
          listvertices_temp.push_back(*inspectLV);
        }
        ++inspectLV;
        ++j;
        if ((*inspectLV)->get_position()->y
                  != environment->get_center()->y - environment->get_height() / 2){
            listvertices_temp.push_back(
                  VerticesPtr(
                      new Vertices(
                          PointPtr(
                              new Point(center_temp->x + size_x / 2,
                                  center_temp->y - size_y / 2)),
                          RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
            listvertices_temp.push_back(*inspectLV);
        }
        break;
      }
    } else {
      if ((*inspectLV)->get_position()->y
          == environment->get_center()->y - environment->get_height() / 2) {
    	std::cout<<"Into"<<std::endl;
    	listvertices_temp.push_back(*inspectLV);
        ++inspectLV;
        ++j;
        continue;
      }
      inspectLVT = --listvertices_temp.end();
      if (((*inspectLV)->get_position()->y == (*inspectLVT)->get_position()->y)) {
        listvertices_temp.push_back(*inspectLV);
        ++inspectLV;
        ++j;
        continue;
      }
      for (inspectLVT = listvertices_temp.begin();
          inspectLVT != listvertices_temp.end(); ++inspectLVT) {
        std::cout << "V current " << (*inspectLV)->get_position()->y
            << std::endl;
        std::cout << "V temp " << (*inspectLVT)->get_position()->y << std::endl;
        if ((*inspectLV)->get_position()->y
            == (*inspectLVT)->get_position()->y) {
          std::cout << (*inspectLV)->get_position()->y << std::endl;
          break;
        }
        vertices_previous = *inspectLVT;
      }
      if ((*inspectLV)->upon_compared_center()) {
        vertices_previous = *inspectLVT;
        ++inspectLVT;
      }
      size_x = (*inspectLV)->get_position()->x
          - vertices_previous->get_position()->x;
      size_y = (*inspectLVT)->get_position()->y
          - vertices_previous->get_position()->y;
      center_temp = PointPtr(
          new Point(vertices_previous->get_position()->x + size_x / 2,
              vertices_previous->get_position()->y + size_y / 2));
      list_space.push_back(SpacePtr(new Space(center_temp, size_x, size_y)));

      // Remove : two vetices space left
      listvertices_temp.remove(vertices_previous);
      listvertices_temp.remove(*inspectLVT);
      if ((*inspectLV)->upon_compared_center()) {
        listvertices_temp.push_back(
            VerticesPtr(
                new Vertices(
                    PointPtr(
                        new Point(center_temp->x + size_x / 2,
                            center_temp->y + size_y / 2)),
                    RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
        std::cout << center_temp->x + size_x / 2 << center_temp->y + size_y / 2
            << std::endl;
      } else {
        listvertices_temp.push_back(
            VerticesPtr(
                new Vertices(
                    PointPtr(
                        new Point(center_temp->x + size_x / 2,
                            center_temp->y - size_y / 2)),
                    RectanglePtr(new Rectangle(center_temp, size_x, size_y)))));
      }
      if ((*inspectLV)->get_position()->x
          == environment->get_center()->x + environment->get_width() / 2) {
        ++inspectLV;
        ++j;
      }
    }
  }