#include "../include/mcts_random.h"


using namespace std;


MCTS::MCTS(ros::NodeHandle node, State &state_, Coordinate &goal_) {
    // Generate ROS nodes
    PubPath = node.advertise<nav_msgs::Path>("mcts_path", 1); // not used now...
    
    node.getParam("DEBUG", DEBUG);

    // Take MCTS parameters
    node.getParam("c", c);
    node.getParam("reward_c", reward_c);
    node.getParam("MAX_DEPTH", MAX_DEPTH);
    node.getParam("MAX_ITER", MAX_ITER);
    node.getParam("MAX_TIME", MAX_TIME);
    node.getParam("dt", dt);
    node.getParam("gamma", gamma);

    if(DEBUG) {
        cout << "Generate MCTS Agent" << endl;
        cout << "Debugging Mode" << endl;
    }

    goal = goal_;
    
    node_list.reserve(1000000); // allocate enough memory to global node list
    node_list.clear();

    // Root node initialize
    InitNode(root, state_);
    root.is_root = true;
    root.depth = 0;
    node_list.push_back(root); // save node at global variable

    current_node = &node_list[0];
    node_list[0].parent = &node_list[0]; // root node takes the parent itself
}

MCTS::~MCTS() {}

tuple<double, double, nav_msgs::Path> MCTS::main() {
    if (DEBUG)
        cout << "Coordinate information of root node: (" << root.current_state.pos.x << ", " << root.current_state.pos.y << ", " << root.current_state.pos.th << ")" << endl;

    ros::Time time_flag = ros::Time::now();
    if (DEBUG) {
        cout << "MCTS Start" << endl;
    }
    while ((ros::Time::now() - time_flag).toSec() < MAX_TIME && backup_count < MAX_ITER) { // tree search until time limit
        if(DEBUG) {
            cout << "\n\nThis value must be true... is root? : " << current_node->is_root << endl;
        }
    
        nav_msgs::Path mcts_path;
        
        while(!(current_node->is_leaf)) {
            add_mcts_path(current_node, mcts_path);
            current_node = Selection(current_node);
        }
        if(DEBUG) {
            cout << "\n\nleaf node found, idx info : " << current_node->node_idx << ", depth info : " << current_node->depth << endl;
        }

        if(current_node->depth < MAX_DEPTH) {
            Expansion(current_node);

            // Random Expansion
            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<int> dis(0, current_node->children.size()-1);
            int rnd_idx = dis(gen);
            current_node = current_node->children[rnd_idx];
            if(DEBUG) {
                cout << "\n\nSelected node idx : " << current_node->node_idx << endl;
            }
        }

        PubPath.publish(mcts_path);

        double r = Evaluation(current_node);

        BackUp(current_node, r);

        backup_count++;
    }

    if(DEBUG) {
        cout << "Finished Tree Search, current node should be root... is root? : " << current_node->is_root << endl;
    }


    cout << "Tree Search Result\n";
    NODE* best_child;
    nav_msgs::Path mcts_path_opt;
    while(!current_node->is_leaf) {
        add_mcts_path(current_node, mcts_path_opt);

        current_node = print_child_info(current_node);
        if(current_node->parent->is_root) {
            best_child = current_node;
        }
    }

    tuple<int, int> last_action_idx_ = best_child->last_action_idx;
    double s = root.current_state.s + steer_cand[get<0>(last_action_idx_)];
    double v = root.current_state.v + vel_cand[get<1>(last_action_idx_)];
    if (s * RAD2DEG > MAX_S) {
        s = MAX_S * DEG2RAD;
    } else if (-MAX_S > s * RAD2DEG) {
        s = -MAX_S * DEG2RAD;
    }
    if(v * MPS2KMPH > MAX_V) {
        v = MAX_V * KMPH2MPS;
    } else if(v * MPS2KMPH < -MAX_V) {
        v = -MAX_V * KMPH2MPS;
    }
    

    double x_, y_, th_;
    tie(x_, y_, th_) = localization(node_list [0].current_state.pos, goal);
    cout << "Root info - x : " << x_ << ", y : " << y_ << ", th : " << th_ * RAD2DEG << "\n";
    cout.precision(4);
    cout << "Control Command { Steer : " << s * RAD2DEG << "[deg] , Velocity : " << v * MPS2KMPH << "[km/h] }\n"
    << "Calc time(ms) : " << (ros::Time::now() - time_flag).toSec() * 1000
    << ", # backups : " << backup_count
    << ", # nodes generated : " << global_idx << "\n\n";

    return make_tuple(s, v, mcts_path_opt);
}


void MCTS::InitNode(NODE& node, State state) {
    node.n = 0;
    node.q = 0;
    node.is_root = false;
    node.is_leaf = true;
    node.node_idx = global_idx++;
    node.current_state = state;
    node.is_terminal = false;
}


NODE* MCTS::Selection(NODE* node) {
    if (DEBUG) {
        cout << "\n\n============ Selection ============" << endl;
    }
    
    int bestidx = 0;
    for(int i = 0; i < node->children.size(); i++) {
        if(DEBUG) {
            cout << "selection candidate idx : " << node->children[i]->node_idx;
            cout << ", candidate puct : " << PUCT(node->children[i]) << endl;
        }
        if (PUCT(node->children[i]) > PUCT(node->children[bestidx])) {
            bestidx = i;
        }
    }

    if (DEBUG) {
        cout << "Best node index is : " << node->children[bestidx]->node_idx << endl;
    }
    return node->children[bestidx];
}


void MCTS::Expansion(NODE* node) {
    if (DEBUG) {
        cout << "\n\n============ Expansion ============" << endl;
    }

    node->is_leaf = false;

    // If the node is terminal state, it acts as absorbing state, generating only one child node
    if(node->is_terminal) {
        NODE child_node = Move(node, 0.0, 0.0); // local_variable
        child_node.parent = node;
        child_node.depth = node->depth + 1;
        child_node.last_action_idx = make_tuple(1, 1);
        child_node.p = 1 / 15.0;
        node_list.push_back(child_node);
        node->children.push_back(&node_list[global_idx - 1]);
        return;
    }
    for(int j = 0; j < vel_cand.size(); j++) {
        for(int i = 0; i < steer_cand.size(); i++) {
            double ds = steer_cand[i];
            double v = vel_cand[j];

            if(abs(node->current_state.s + ds) * RAD2DEG > MAX_S) {
                continue;
            }
            NODE child_node = Move(node, ds, v); // local_variable
            child_node.parent = node;
            child_node.depth = node->depth + 1;
            child_node.last_action_idx = make_tuple(i, j);
            child_node.p = 1 / 15.0;

            node_list.push_back(child_node); // save node at global variable
            if(DEBUG) {
                cout << child_node << endl;
            }
            node->children.push_back(&node_list[global_idx - 1]);
        }
    }
}


double MCTS::Evaluation(NODE* node) {
    if (DEBUG)
        cout << "\n\n========== Evaluation ==========" << endl;
        
    double x, y, th;
    tie(x, y, th) = localization(node->current_state.pos, goal);
    double dist = sqrt(pow(x, 2) + pow(y, 2));
    double v;

    v = 0.5 * ((1 - reward_c) * rational_function(dist) + reward_c * rational_function(abs(3 * th))) + 0.4;
    // v = 0.5 * (1* func(dist)) + 0.4;
    // if(x < 0.0) {
    //     v += -0.4;
    //     node->current_state.v = 0;
    //     node->is_terminal = true;
    // }
    // if(x > 11.7) {
    //     v += -0.4;
    //     node->current_state.v = 0;
    //     node->is_terminal = true;
    // } 
    if(abs(y) < 0.1 && dist < 0.5 && abs(th) < 10 * DEG2RAD) {
        v += 0.1;
        node->current_state.v = 0;
        node->is_terminal = true;
    }
    

    v = (v - 0.5) * 2;
    if (DEBUG)
        cout << "Evaluated value : " << v << endl;

    return v;
} 


void MCTS::BackUp(NODE* node, double reward) {
    if (DEBUG) {
        cout << "\n\n========== Backup ==========" << endl;
        cout << "node idx : " << node->node_idx << endl;
        cout << "is root node? : " << node->is_root << endl;
        cout << "q before backup : " << node->q << endl;
        cout << "current node visit count : " << node->n << endl;
    }

    // update the node info
    double q_sum = node->q * node->n + reward;
    node->n += 1;
    node->q = q_sum / node->n;

    if(DEBUG) {
        cout << "q after backup : " << node->q << endl;
    }

    if(!(node->is_root)){
        if(DEBUG) {
            cout << "next backup node idx : " << node->parent->node_idx << endl;
        }
        current_node = node->parent;
        BackUp(current_node, gamma * reward);
    }
}


NODE MCTS::Move(NODE* node, double ds, double dv) { // give an action as input and return new node with next position
    // Assuming constant v, s while moving
    double new_s = node->current_state.s + ds;
    double new_v = node->current_state.v + dv;

    // Physical constraint of the car
    if(new_s * RAD2DEG > MAX_S) {
        new_s = MAX_S * DEG2RAD;
    } else if(new_s * RAD2DEG < -MAX_S) {
        new_s = -MAX_S * DEG2RAD;
    }
    if(new_v * MPS2KMPH > MAX_V) {
        new_v = MAX_V * KMPH2MPS;
    } else if(new_v * MPS2KMPH < -MAX_V) {
        new_v = -MAX_V * KMPH2MPS;
    }

    // Calculate wheel angle of the car
    double wheel_angle = 1 / 2.0 * (steering_outerwheelangle_ratio + steering_innerwheelangle_ratio) * new_s;

    // Next point calculation with interval dt
    double dx = new_v * dt * cos(node->current_state.pos.th);
    double dy = new_v * dt * sin(node->current_state.pos.th);
    double dth = (new_v * dt / vehicle_length) * tan(-wheel_angle);

    State next_state;
    next_state.pos.x = node->current_state.pos.x + dx;
    next_state.pos.y = node->current_state.pos.y + dy;
    next_state.pos.th = remainder((node->current_state.pos.th + dth), 2 * M_PI);
    next_state.s = new_s;
    next_state.v = new_v;

    if(DEBUG) {
        cout << "Move Info" << endl;
        cout << "x = " << node->current_state.pos.x << ", y = " << node->current_state.pos.y << ", th = " << node->current_state.pos.th * RAD2DEG << ", s = " << node->current_state.s << ", v = " << node->current_state.v << endl;
        cout << "dx = " << dx << ", dy = " << dy << ", dth = " << dth * RAD2DEG << ", ds = " << ds << ", dv = " << dv << endl;
        cout << "x = " << next_state.pos.x << ", y = " << next_state.pos.y << ", th = " << next_state.pos.th * RAD2DEG << ", s = " << next_state.s << ", v = " << next_state.v << endl;
    }

    NODE next_node;
    InitNode(next_node, next_state);
    return next_node;
}


double MCTS::PUCT(NODE* node) {
    return node->q + c * node->p * sqrt(node->parent->n / (1.0 + node->n));
}


NODE* MCTS::print_child_info(NODE* node) {
    int bestidx = 0;
    cout << "Current node depth: " << node->depth << ", Print child nodes info.\n";

    for(int i = 0; i < node->children.size(); i++) {
        cout.precision(2);
        cout << "Node index: " << node->children[i]->node_idx
        << ", dv : " << setw(2) << vel_cand[get<1>(node->children[i]->last_action_idx)] * MPS2KMPH << "[km/h]"
        << ", ds : " << setw(3) << steer_cand[get<0>(node->children[i]->last_action_idx)] * RAD2DEG << "[deg]";
        cout.precision(5);
        cout << ", # Visit : " << setw(5) << node->children[i]->n
        << ", Q-value : " << setw(10) << node->children[i]->q
        << ", PUCT : " << setw(10) << PUCT(node->children[i]) << endl;

        if (node->children[i]->n > node->children[bestidx]->n) {
            bestidx = i;
        }
    }
    return node->children[bestidx];
}


void MCTS::add_mcts_path(NODE* node, nav_msgs::Path &mcts_path) {
    mcts_path.header.frame_id = point.header.frame_id = "map";
    mcts_path.header.stamp = point.header.stamp = ros::Time::now();

    point.pose.position.x = node->current_state.pos.x;
    point.pose.position.y = node->current_state.pos.y;
    point.pose.position.z = 0.5;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(node->current_state.pos.th);
    point.pose.orientation = odom_quat;
    mcts_path.poses.push_back(point);
}

