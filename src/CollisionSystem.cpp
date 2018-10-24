#include "CollisionSystem.h"
#include "extern.h"

using namespace lm;

//nothing to initialise so far
void CollisionSystem::init() {
    
}

void CollisionSystem::update(float dt) {
    //necesary variables
    lm::vec3 col_point;
    
    //reset all collisions every frame
    auto& colliders = ECS.getAllComponents<Collider>();
    for (auto& col : colliders){
        col.colliding = false;
        col.collision_distance = 10000000.0f;
        col.other = nullptr;
    }

    //test ray-box collision. This works by looping over ray colliders. For each one, we loop over box colliders
    //test collision between ray and box, updating collision distance for each collision found
    //then for future collision tests only look as far as existing stored collision distance
    for (size_t i = 0; i < colliders.size(); i++) {

        //if collider is ray
        if (colliders[i].collider_type == ColliderTypeRay) {
            
            //test all other colliders
            for (size_t j = 0; j < colliders.size(); j++) {
                if (j == i) continue; // no self-test
                
                //if box
                if (colliders[j].collider_type == ColliderTypeBox) {
                    //test collision
                    float col_distance = 0; //temp var to store distance
                    if (intersectSegmentBox(colliders[i], //the ray
                                            colliders[j], //the box
                                            col_point, //reference to collision point
                                            col_distance, //reference to collision distance
                                            colliders[i].collision_distance)){ //only look as far as current nearest collider
                        colliders[i].colliding = colliders[j].colliding = true;
                        colliders[i].collision_point = colliders[j].collision_point = col_point;
                        colliders[i].collision_distance = colliders[j].collision_distance = col_distance;
                        std::cout << "collision at " << col_point.x << ", " << col_point.y << ", " << col_point.z << ", " << col_distance << "\n";
                    }
                }
            }
        }
    }
}

// Calculates whether a Ray collider (treated as a segment with a finite distance)
// collides with a box collider.
// - ray: reference to ray collider object
// - box: reference to the box collider object
// - col_point: reference to an empty vec3 which will be updated with the collision point
// - reference to a float which will be updated with the distance to the nearest collider
// - optional variable which specifies the maximum distance along ray which to search
bool CollisionSystem::intersectSegmentBox(Collider& ray, Collider& box, lm::vec3& col_point, float& col_distance, float max_distance) {
    //the general approach of this function is as follows
    // - transform ray and box into world space and apply any offsets
    // - create six planes of box
    // - calculate collision of ray with each plane
    // note that there is an inherent optimization in that the intersectSegmentQuad
    // function already discards cases where ray points in same direction as quad
    // normal, so in fact we only test collisions for maximum 3 faces
    

	//TODO:
	// - get local points of ray/segment start point (use local_center),
	//   and 8 points of box (use local_center and local_halfwidths

	// - get model matrices of ray/segment and box, and 'all transforms' vector 
	//   (to pass to getGlobalMatrix() function)
	// - get global position of ray/segment start point all 8 points of box
	//   (a, b, c, d, e, f, g, h)

	// - ray/segment direction must be rotated according to model matrix
	//   WITHOUT the scale and translate component. We usually do this by
	//   cutting the 4x4 matrix to a 3x3 matrix (removing the final column)
	//   with translate vector, and then calculating the inverse-transpose,
	//   which effectively 'normalises' the matrix (set the scale component to 1).
	//	 However our Linear Math library does not contain a definition for mat3,
	//   so we just set the 12th, 13th, and 14th component of model matrix to 0
	//	 (the translate component) and calculate the inverse-transpose of this matrix
	//   this will give us a matrix which rotates, but does not translate or scale
	//
	//	indices of mat4.m: 12, 13, 14 are the translate component
	//
	//  0	4	8	Tx	
	//	1	5	9	Ty
	//	2	6	10	Tz
	//	3	7	11	15
	//

	// - normalise new direction 
	// - scale normalized direction by ray/segment max_distance
	// - far point of segment is start_point + scaled_direction
    
	// - pass segment point and quad point to intersectSegmentQuad function
	//   pass the col_point float BY REFERENCE to obtain distance along
	//	 ray direction of collision point
	// - quads are:
    //   abcd; dcgh, hgfe, efba, adhe, bfgc

    // - return true if collision, false if none

    return false;
}

// Test for collision between a segment PQ and a directed, plane quad (ABDC)
// Approach is to do two ray-in-triangle tests for triangles of quad
// see pages 188 - 190 for Real Time Collision Detection (Erikson) for more info
// - p: ray start
// - q: ray end
// - abcd: points of quad, given counterclockwise
bool CollisionSystem::intersectSegmentQuad(lm::vec3 p, lm::vec3 q, lm::vec3 a, lm::vec3 b, lm::vec3 c, lm::vec3 d, lm::vec3& r) {
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 qp = p - q;
    
    vec3 n = ab.cross(ac); //triangle normal
    
    //angle between normal and the segment. if <= 0 segment is parallel or points away
    float den = qp.dot(n);
    if (den <= 0.0f) return 0;
    
    //intersection with plane of triangle, compute value of t
    vec3 ap = p - a;
    float t = ap.dot(n);
    if (t < 0.0f) return false;
    if (t > den) return false; //only for segment, comment for ray test
    
    //now test against individual triangles using barycentric coords
    vec3 e = qp.cross(ap); //orthogonal to plane and segment
    
    //abc
    bool abc_collide = true;
    float v = ac.dot(e);
    if (v < 0.0f || v > den) {
        abc_collide = false;
    };
    float w = -(ab.dot(e));
    if (w < 0.0f || v + w > den) {
        abc_collide = false;
    }
    if (abc_collide){
        //intersect, get ray parameter distance
        float ood = 1.0f / den;
        v *= ood;
        w *= ood;
        float u = 1.0f - v - w;
        r = a*u + b*v + c*w;
        return true;
    }
    
    //acd
    vec3 ad = d - a;
    v = ad.dot(e);
    if (v < 0.0f || v > den) {
        return false;
    };
    w = -(ac.dot(e));
    if (w < 0.0f || v + w > den) {
        return false;
    }
    //intersect acd, get ray parameter distance
    float ood = 1.0f / den;
    v *= ood;
    w *= ood;
    float u = 1.0f - v - w;
    r = a*u + c*v + d*w;
    return true;
}

//See page 190 of Real Time Collision Detection
bool CollisionSystem::intersectSegmentTriangle(lm::vec3 p, lm::vec3 q, lm::vec3 a, lm::vec3 b, lm::vec3 c) {
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 qp = p - q;
    
    vec3 n = ab.cross(ac); //triangle normal
    
    //demoninator (angle between normal and line). if <= 0 segment is parallel or points away
    float d = qp.dot(n);
    if (d <= 0.0f) return 0;
    
    //intersection with plane of triangle, compute value of t
    vec3 ap = p - a;
    float t = ap.dot(n);
    if (t < 0.0f) return false;
    if (t > d) return false; //only for segment, comment for ray test
    
    //barycentric coords
    vec3 e = qp.cross(ap);
    float v = ac.dot(e);
    if (v < 0.0f || v > d) return false;
    float w = -(ab.dot(e));
    if (w < 0.0f || v + w > d) return false;
    
    return true;
}

//LINE not segment
//See page 188 of Real Time Collision Detection
bool CollisionSystem::intersectLineQuad(lm::vec3 p, lm::vec3 q, lm::vec3 a, lm::vec3 b, lm::vec3 c, lm::vec3 d, lm::vec3& r) {
    vec3 pq = q - p;
    vec3 pa = a - p;
    vec3 pb = b - p;
    vec3 pc = c - p;
    
    vec3 m = pc.cross(pq);
    float v = pa.dot(m);
    if (v >= 0.0f) {
        //test triangle abc
        float u = -pb.dot(m);
        if (u < 0.0f) return false;
        float w = (pq.cross(pb)).dot(pa);
        if (w < 0.0f) return false;
        //if true compute r from barycentric coords
        float denom = 1.0f / (u + v + w);
        u *= denom; v *= denom; w *= denom;
        r = a*u + b*v + c*w;
    } else {
        //test triangle dac
        vec3 pd = d - p;
        float u = pd.dot(m);
        if (u < 0.0f) return false;
        float w = (pq.cross(pa)).dot(pd);
        if (w < 0.0f) return false;
        v = -v;
        //if true compute r from barycentric coords
        float denom = 1.0f / (u + v + w);
        u *= denom; v *= denom; w *= denom;
        r = a*u + d*v + c*w;
    }
    return true;
}

