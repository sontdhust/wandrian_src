####2D-Spiral-STC Algorithm:
__Sensors__: A position and orientation sensor. A 4-neighbors obstacle detection sensor.

__Input__: A starting cell _S_, but no a priori knowledge of the environment.
Recursive function: __STC1__(_w_, _x_), where _x_ is the current cell and _w_ the parent cell in the spanning tree.

Initialization: Call __STC1__(_Null_, _S_), where _S_ is the starting cell.

__STC1__(_w_, _x_):

1. Mark the current cell _x_ as an old cell.
2. __While__ _x_ has a new obstacle-free neighboring cell:

      - Scan for the first new neighbor of _x_ in counterclockwise order, starting with the parent cell _w_. Call this neighbor _y_.
      - Construct a spanning-tree edge from _x_ to _y_.
      - Move to a subcell of _y_ by following the right-side of the spanning tree edges.
      - Execute __STC1__(_x_, _y_).
      
      __End of while loop__.


3. __If__ _x_ = _S_, move back from _x_ to a subcell of _w_ along the right-side of the spanning tree edges.
4. __Return__. (End of __STC1__(_w_, _x_)).