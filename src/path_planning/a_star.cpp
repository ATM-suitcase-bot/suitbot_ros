#include "a_star.h"

Node::Node(){
    g = -1;
    h = -1;
}

Node::Node(int x_idx_, int y_idx_){
    indices.first = x_idx_;
    indices.second = y_idx_;
    g = -1;
    h = -1;
}

Node::Node(std::pair<int, int> p_idx){
    indices.first = p_idx.first;
    indices.second = p_idx.second;
    g = -1;
    h = -1;
}


AugmentedNode::AugmentedNode(){}

AugmentedNode::AugmentedNode(Node *node_){
    node = node_;
}