#ifndef MCTS_RANDOM_H_
#define MCTS_RANDOM_H_


#include "utils.h"


using namespace std;


// Node Implementation
struct NODE
{
    NODE* parent; // node of the last time step
    vector<NODE*> children; // node of the next time step

    State current_state; // current node state
    
    int node_idx; // incrementally assigned index, zero for root node
    int n; // visit count, updated at backup stage
    double q; // expected cumulated reward, updated at backup stage
    double p; // prior, in this configure the network is missing, set uniform
    bool is_root; // true for root node
    bool is_leaf; // true for leaf node
    int depth; // tree depth, zero for root
    tuple<int, int> last_action_idx; // index of (steer, velocity) action set
    bool is_terminal; // true if the current state satisfies the terminal condition

    NODE() {
        children.reserve(9);
    }

    friend ostream& operator<<(ostream &o, const NODE &n) {
        o << "Node Info" << endl
        << "Node index : " << n.node_idx << endl
        << "Parent node index : " << n.parent->node_idx << endl
        << "State " << n.current_state << endl
        << "Visit Count : " << n.n << endl
        << "Prior : " << n.p << endl
        << "Value : " << n.q << endl
        << "Root/Leaf : " << n.is_root << " / " << n.is_leaf << endl
        << "Node Depth : " << n.depth << endl
        << "Action Taken : (" << get<0>(n.last_action_idx) << ", " << get<1>(n.last_action_idx) << ")" << endl
        << "Termianl state : " << n.is_terminal << endl;
        return o;
    }
};


class MCTS
{
    public: 
        MCTS(ros::NodeHandle node, State &state_, Coordinate &goal_);
        ~MCTS();

        tuple<double, double, nav_msgs::Path> main();
        /**
         * @brief executes the MCTS loop
         * @return next time step control command(steering, velocity) and mostly visited mcts path for visualizing
        */

    private:
        // ROS variables
        ros::Publisher PubPath;
        geometry_msgs::PoseStamped point;


        // DEBUG util
        bool DEBUG = false;


        // Parameters
        const double MAX_S = 540; // deg
        const double MAX_V = 5; // km/h
        const double steering_wheelangle_ratio = 70 / 540.0; // one-to-one match between steering and corresponding wheel angle
        const double vehicle_length =  2.8325145696692897; // using wheel base of 2020 Benz
        double c = 1.4; // parameter of PUCT
        double reward_c = 1.0; // parameter of reward function, reward weight between angle and euclidian
        int MAX_DEPTH = 5; // maximum depth of MCTS Tree
        int MAX_ITER = 1000; // maximum backups of MCTS Tree
        double MAX_TIME = 0.04; // MCTS process terminate within max time
        double dt = 0.25; // time steps interval between nodes
        double gamma = 0.98; // n-step reward decay
        vector<double> steer_cand = {-20 * DEG2RAD, 0, 20 * DEG2RAD};
        vector<double> vel_cand = {-0.2 * KMPH2MPS, 0, 0.2 * KMPH2MPS};
        int backup_count = 0; // for counting how many tree searches are executed
        int global_idx = 0; // global index of the nodes


        // Nodes
        NODE root;
        vector<NODE> node_list;
        NODE* current_node;


        // Variables
        Coordinate goal;


        // MCTS Process
        NODE* Selection(NODE* node);
        /**
         * @brief selects the node with highest PUCT among the child nodes
         * @return best child node pointer
        */

        void Expansion(NODE* node);
        /**
         * @brief generates child nodes using combination of steer and velocity candidates
         * @return nothing, but add nodes in global node list
        */

        double Evaluation(NODE* node);
        /**
         * @brief calculates reward of given node
         * @return scalar reward
        */

        void BackUp(NODE* node, double reward);
        /**
         * @brief updates q, n of the node with given reward
         * @return nothing, but moves current_node to parent node for next backup calculation
        */


        // Utils
        void InitNode(NODE& node, State state);
        /**
         * @brief initializes the node
         * @return nothing, but changes the value of the node to default
        */

        NODE Move(NODE* node, double ds, double dv);
        /**
         * @brief using kinematics, generates the node when given parent node state and executed action
         * @return the node after move
        */

        double PUCT(NODE* node);
        /**
         * @brief calculate PUCT of the node
         * @return scalar PUCT value
        */

        NODE* print_child_info(NODE* node);
        /**
         * @brief print child node visit count, PUCT, etc., for convenience of debugging
         * @return best child node pointer
         */

        void add_mcts_path(NODE* node, nav_msgs::Path &mcts_path);
        /**
         * @brief add current node state to mcts_path
         * @bug does not visualize well, especially on real time control, so disabled now
        */

};


#endif
