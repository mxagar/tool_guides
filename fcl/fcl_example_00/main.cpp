#include "fcl/fcl.h"
#include <iostream>

using namespace fcl;

int main() {

    // 1)
    // Before starting the proximity computation,
    // we need first to set the geometry and transform for the objects involving in computation.
    // The geometry of an object is represented as a mesh soup, which can be set as follow

    // set mesh triangles and vertice indices
    std::vector<Vector3f> vertices;
    std::vector<Triangle> triangles;
    // code to set the vertices and triangles
    // ...
    // BVHModel is a template class for mesh geometry, for default OBBRSS template
    // is used
    typedef BVHModel<OBBRSSf> Model;
    std::shared_ptr<Model> geom = std::make_shared<Model>();
    // add the mesh data into the BVHModel structure
    geom->beginModel();
    geom->addSubModel(vertices, triangles);
    geom->endModel();
    

    // 2)
    // The transform of an object includes the rotation and translation

    // R and T are the rotation matrix and translation vector
    Matrix3f R;
    Vector3f T;
    // code for setting R and T
    //...
    // transform is configured according to R and T
    Transform3f pose = Transform3f::Identity();
    pose.linear() = R;
    pose.translation() = T;

    // 3)
    // Given the geometry and the transform,
    // we can also combine them together to obtain a collision object instance;
    // here is an example:

    CollisionObjectf* obj = new CollisionObjectf(geom, pose);

    return 0;
}