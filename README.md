### La Salle - Universitat Ramon Llull : Master in Advanced Videogame Development
## Engine Programming - 05-CollisionSystem

### TODO 

#### in CollisionSystem.cpp
 - get local points of ray/segment start point (use local_center),
   and 8 points of box (use local_center and local_halfwidths)

 - get model matrices of ray/segment and box, and 'all transforms' vector 
   (to pass to getGlobalMatrix() function)
 - get global position of ray/segment start point all 8 points of box
   (a, b, c, d, e, f, g, h)

 - ray/segment direction must be rotated according to model matrix
   WITHOUT the scale and translate component. We usually do this by
   cutting the 4x4 matrix to a 3x3 matrix (removing the final column)
   with translate vector, and then calculating the inverse-transpose,
   which effectively 'normalises' the matrix (set the scale component to 1).
	 However our Linear Math library does not contain a definition for mat3,
   so we just set the 12th, 13th, and 14th component of model matrix to 0
	 (the translate component) and calculate the inverse-transpose of this matrix
   this will give us a matrix which rotates, but does not translate or scale

 - normalise new direction 
 - scale normalized direction by ray/segment max_distance
 - far point of segment is start_point + scaled_direction

 - pass segment point and quad point to intersectSegmentQuad function
   pass the col_point float BY REFERENCE to obtain distance along
	 ray direction of collision point
 - quads are:
   abcd; dcgh, hgfe, efba, adhe, bfgc

 - return true if collision, false if none
