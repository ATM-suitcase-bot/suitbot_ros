/**
 * @file planner.h
 *
 * @brief A* planner
 *
 * @date 07/15/2021
 *
 * @author Tina Tian (yutian)
 */

#include "planner_node.h"


#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


struct hash_pair
{
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2> &p) const
    {
        std::size_t seed = 0;
        seed ^= hash<T1>{}(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hash<T2>{}(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

int PlannerNode::a_star_planner()
{
    // init
    banked_steps.clear();
    // start is cell where the robot is in now
    Node *start_node = new Node(start_x_idx, start_y_idx);
    start_node->g = 0;
    start_node->predecessor = start_node;
    // goal
    Node *goal_node = new Node(goal_x_idx, goal_y_idx);
    cout << "start: " << start_x_idx << ", " << start_y_idx << endl;
    cout << "goal : " << goal_x_idx << ", " << goal_y_idx << endl;
    if (goal_node->indices == start_node->indices)
    {
        cout << "arrived at goal" << endl;
        return 0;
    }

    // TODO modify compute and fill in the h value based on the goal
    compute_h(start_node, goal_node);

    bool found_soln = false;
    priority_queue<AugmentedNode> OPEN;
    unordered_map<pair<int, int>, Node *, hash_pair> EXPANDED;
    unordered_map<pair<int, int>, Node *, hash_pair> pq_set;

    OPEN.push(AugmentedNode(start_node));
    // while(sgoal is not expanded and OPEN ≠ 0)
    while (OPEN.size() != 0)
    {
        // remove s with the smallest [f(s) = g(s)+h(s)] from OPEN;
        AugmentedNode aug_node = OPEN.top();
        OPEN.pop();
        // insert s into CLOSED;
        EXPANDED[aug_node.node->indices] = aug_node.node;
        pq_set.erase(aug_node.node->indices);
        // if goal is expanded, we stop
        if (aug_node.node->indices == goal_node->indices)
        {
            found_soln = true;
            // TODO
            goal_node = aug_node.node;
            break;
        }
        // for every successor s’ of s such that s’ not in CLOSED
        vector<pair<int, int>> successors;
        get_successors(aug_node, successors);

        for (size_t i = 0; i < successors.size(); i++)
        {
            pair<int, int> s_idx = successors.at(i);
            //  check if in expanded
            if (EXPANDED.find(s_idx) != EXPANDED.end())
                continue;
            // check if in open
            auto successor_lookup = pq_set.find(s_idx);
            Node *successor_node;
            bool successor_in_open = true;
            if (successor_lookup == pq_set.end())
            {
                successor_in_open = false;
                successor_node = new Node(s_idx);
                successor_node->predecessor = aug_node.node;
            }
            else
            {
                successor_node = successor_lookup->second;
            }
            if (successor_node->h == -1)
                compute_h(successor_node, goal_node);
            // if g(s’) > g(s) + c(s,s’)
            double cost = compute_cost(aug_node.node, successor_node);
            if (successor_node->g == -1 || successor_node->g > aug_node.node->g + cost)
            {
                // g(s’) = g(s) + c(s,s’);
                successor_node->g = aug_node.node->g + cost;
                successor_node->predecessor = aug_node.node;
                // insert s’ into OPEN;
                if (!successor_in_open)
                {
                    OPEN.push(AugmentedNode(successor_node));
                    pq_set[s_idx] = successor_node;
                }
            }
        }
    }

    if (found_soln)
    {
        backtrack(goal_node);
        cout << "done backtrack" << endl;
        return 0;
    }
    else
    {
        cout << " soln not found??" << endl;
        return -1;
    }
}

// get the successors of a node
void PlannerNode::get_successors(AugmentedNode &aug_node, vector<pair<int, int>> &successors)
{
    int row = aug_node.node->indices.second;
    int col = aug_node.node->indices.first;
    for (int i = -1; i < 2; i++)
    {
        int row_new = row + i;
        for (int j = -1; j < 2; j++)
        {
            int col_new = col + j;
            if ((row_new != row || col_new != col) && row_new >= 0 && row_new < map.rows && col_new >= 0 && col_new < map.cols)
            {
                if (map.occupancy_map[row_new][col_new] == FREE)
                {
                    pair<int, int> succ;
                    succ.first = col_new;
                    succ.second = row_new;
                    successors.push_back(succ);
                }
            }
        }
    }
}

void PlannerNode::compute_h(Node *node, Node *goal_node)
{
    // goals have h=0
    if (node->indices == goal_node->indices)
    {
        node->h = 0;
        return;
    }
    double cx = (double)(node->indices.first); // simply use their x y indices 
    double cy = (double)(node->indices.second);
    double gcx = (double)(goal_node->indices.first);
    double gcy = (double)(goal_node->indices.second);
    // double dist = sqrt((gcx-cx)*(gcx-cx) + (gcy-cy) * (gcy-cy));
    // using 8-point connection
    double dist = (sqrt(2) - 1) * MIN(abs(cx - gcx), abs(cy - gcy)) + MAX(abs(cx - gcx), abs(cy - gcy));
    node->h = dist;
}


double PlannerNode::compute_cost(Node *n1, Node *n2)
{
    double cx1 = (double)(n1->indices.first); // simply use their x y indices 
    double cy1 = (double)(n1->indices.second);
    double cx2 = (double)(n2->indices.first);
    double cy2 = (double)(n2->indices.second);
    return sqrt((cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2));
}


void PlannerNode::backtrack(Node *goal_node)
{
    Node *curr = goal_node;
    cout << "is null: " << (curr == nullptr) << endl;
    vector<pair<int, int>> waypoint_idx;
    while (1)
    {
        waypoint_idx.push_back(curr->indices);
        if (curr->predecessor != curr)
            curr = curr->predecessor;
        else
            break;
    }
    cout << "waypoint size:" << waypoint_idx.size() << endl;
    pair<int, int> goal = waypoint_idx.at(0);
    reverse(waypoint_idx.begin(), waypoint_idx.end());
    int i = 0;
    for (int i = 0; i < waypoint_idx.size(); i++)
    { // the first thing we pop is goal
        pair<int,int> idx = waypoint_idx[i];
        double cx, cy;
        cx = idx.first * resolution + resolution/2;
        cy = idx.second * resolution + resolution/2;
        pair<double, double> p{cx, cy};
        banked_steps.push_back(p);
        i++;
    }
}