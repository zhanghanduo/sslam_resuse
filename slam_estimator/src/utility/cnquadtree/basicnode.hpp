


// BasicNode represents a standard quadtree node.
//
// It is a basic implementation of the Node interface, the one used in the
// BasicTree implementation of the Quadtree interface.
#include "node.hpp"
#include "common.hpp"

struct BasicNode{
    Node parent_;
    Node children_[4];

    cv::Rect2f rect_;
    Color color_;
    int location_; // node location inside its parent
};