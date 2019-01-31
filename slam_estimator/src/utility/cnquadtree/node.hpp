#include <stdio.h>
#include <list>
#include <thread>
#include <opencv2/core/core.hpp>

typedef char Color;

enum {
    // Black is the color of leaf nodes
    // that are considered as obstructed.
    Black,
    // White is the color of leaf nodes
    // that are considered as free.
    White,
    // Gray is the color of non-leaf nodes
    // that contain both black and white children.
    Gray};

class Node {
public:
    std::list<cv::KeyPoint> keypoints_;
    cv::Rect2f rect_;
    std::shared_ptr<Node> children_[4];
    std::shared_ptr<Node> parent_;
    Color color_;

    enum ChildPosition { TOP_LEFT = 0, TOP_RIGHT, BOT_LEFT, BOT_RIGHT };
};
