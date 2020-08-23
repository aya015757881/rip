#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <thread>
#include <mutex>
#include <random>
#define OVERFLOW_CHECK(index, vec) \
            if (index < 0 || index > vec.size() - 1) { \
                stringstream ss;    \
                ss << "vector overflow: "   \
                    << "vector size = " \
                    << vec.size()   \
                    << ", index = " \
                    << index    \
                    << endl; \
                throw ss.str(); \
            }

using namespace std;

mutex mtx;

class RoutingTableEntry {
public:
    int dest_id;
    int next_hop_id = -1;
    int cost = 16;
    vector<int> gids;
    RoutingTableEntry(int dest);
    void print();
};

RoutingTableEntry::RoutingTableEntry(int dest): dest_id(dest) { }

void RoutingTableEntry::print() {
    cout << "dest = "
        << dest_id
        << ", next hop = "
        << next_hop_id
        << ", cost = "
        << cost
        << "\n";
}

class Node {
public:
    int id;
    vector<int> gids;
    vector<Node*> neighbors;
    vector<RoutingTableEntry> routing_table;

    bool is_new_packet = false;
    int packet = -1;
    int src = -1;
    int dest = -1;
    int from = -1;

    Node(): Node(-1) { }
    Node(int id): id(id) { }
    
    void start_routing();
    void start_simplified_dvmrp();
    void run_routing();
    void run_simplified_dvmrp();
    void insert_routing_table_entry(RoutingTableEntry &entry);
    void print_routing_table();
    bool has_channel_for(Node *node);
    void build_channel_for(Node *node);
    void join_group(int gid);
    void leave_group(int gid);
private:
    Node *get_neighbor(int id);
};

void Node::start_routing() {
    thread t(&Node::run_routing, this);
    t.detach();
}

void Node::start_simplified_dvmrp() {
    thread t(&Node::run_simplified_dvmrp, this);
    t.detach();
}

void Node::run_routing() {

    while (true) {

        lock_guard<mutex> lg(mtx);

        if (!is_new_packet)
            continue;
        
        if (dest < routing_table.size()) {
            // here we do the unicast    
            stringstream ss;
            int next_hop = routing_table[dest].next_hop_id;
            ss << "\n\nnode:\t" << id << "\npacket:\t" << packet
                << "\nsrc:\t" << src << "\ndest:\t" << dest
                << "\nfrom:\t" << from << "\nstate: ";
            if (next_hop == -1)
                ss << "dropped\n";
            else if (next_hop == id)
                ss << "received\n";
            else
                for (size_t i = 0; i < neighbors.size(); ++i)
                    if (neighbors[i]->id == next_hop) {
                        neighbors[i]->is_new_packet = true;
                        neighbors[i]->packet = packet;
                        neighbors[i]->src = src;
                        neighbors[i]->dest = dest;
                        neighbors[i]->from = id;
                        ss << "delivered to next hop node: " << next_hop << "\n";
                        break;
                    }        
            cout << ss.str() << endl;

        } else if (src < routing_table.size()) {

            // here we do the multicast

            // if this node has joined the group identified by dest,
            // print the info out to the console indicating the receival.
            if (find(gids.begin(), gids.end(), dest) != gids.end()) {
                cout << "\n\nnode:\t" << id << "\npacket:\t" << packet
                     << "\nsrc:\t" << src << "\ndest:\t" << dest
                     << "\nfrom:\t" << from << "\nstate:\treceived" << endl;
            }

            // accoring to TRPB, we do not route packet from any
            // path other than the reverse path back to the source
            if (from != routing_table[src].next_hop_id)
                goto DONE;
                
            // links not having been used for routing for once
            // is preserved in this not-used-links cache
            vector<Node*> lcache = neighbors;
            
            for (size_t i = 0; !lcache.empty() && i < routing_table.size(); ++i) {
                auto &entry = routing_table[i];
                vector<int> _gids = entry.gids;
                if (find(_gids.begin(), _gids.end(), dest) == _gids.end())
                    continue;
                
                Node *link = get_neighbor(entry.next_hop_id);

                // the packet has been sent through this link for once
                if (find(lcache.begin(), lcache.end(), link) == lcache.end())
                    continue;

                // according to TRPB, we do not route packet through non-child links
                int self2src_cost = routing_table[src].cost;
                int link2src_cost = link->routing_table[src].cost;
                if (self2src_cost > link2src_cost ||
                    self2src_cost == link2src_cost && id > link->id)
                    continue;

                // if control should reach in here, send the packet out through this link
                link->is_new_packet = true;
                link->packet = packet;
                link->src = src;
                link->dest = dest;
                link->from = id;

                // mark the link as used by removing it from the not-used-links cache
                lcache.erase(find(lcache.begin(), lcache.end(), link));
            }
        }
DONE:
        is_new_packet = false;
    }
}

void Node::run_simplified_dvmrp() {

    for (size_t i = 0; i < routing_table.size(); i = i < routing_table.size() - 1 ? i + 1 : 0) {

        lock_guard<mutex> lg(mtx);
        auto &this_entry = routing_table[i];

        if (this_entry.dest_id == id) {
            this_entry.next_hop_id = id;
            this_entry.gids = gids;
            this_entry.cost = 0;
            continue;
        }

        Node *neighbor = nullptr;
        for (int k = 0; k < neighbors.size(); ++k)
            if (this_entry.dest_id == neighbors[k]->id) {
                neighbor = neighbors[k];
                break;
            }
        if (neighbor) {
            this_entry.next_hop_id = neighbor->id;
            this_entry.gids = neighbor->gids;
            this_entry.cost = 1;
            continue;
        }

        this_entry.next_hop_id = -1;
        this_entry.cost = 16;
        this_entry.gids.clear();

        for (int j = 0; j < neighbors.size(); ++j) {
        
            auto &nbr_entry = neighbors[j]->routing_table[i];
        
            if (nbr_entry.next_hop_id == -1 ||
                nbr_entry.next_hop_id == id ||
                nbr_entry.cost >= 15)
                continue;
        
            if (this_entry.next_hop_id == -1 ||
                nbr_entry.cost + 1 < this_entry.cost) {
                this_entry.next_hop_id = neighbors[j]->id;
                this_entry.cost = nbr_entry.cost + 1;
                this_entry.gids = nbr_entry.gids;
            }
        }
        // print_routing_table();
    }
}

void Node::insert_routing_table_entry(RoutingTableEntry &entry) {
    routing_table.push_back(entry);
}

void Node::print_routing_table() {
    for (int i = 0; i < routing_table.size(); ++i) {
        cout << "node " << id << ": ";
        routing_table[i].print();
        cout << endl;
    }
}

bool Node::has_channel_for(Node *node) {
    return find(neighbors.begin(), neighbors.end(), node) != neighbors.end();
}

void Node::build_channel_for(Node *node) {
    neighbors.push_back(node);
}

void Node::join_group(int gid) {

    lock_guard<mutex> lg(mtx);
    if (find(gids.begin(), gids.end(), gid) == gids.end())
        gids.push_back(gid);
}

void Node::leave_group(int gid) {
    
    lock_guard<mutex> lg(mtx);
    gids.erase(find(gids.begin(), gids.end(), gid));
}

Node *Node::get_neighbor(int id) {

    for (size_t i = 0; i < neighbors.size(); ++i)
        if (id == neighbors[i]->id)
            return neighbors[i];

    return nullptr;
}

class edge {
public:
    unsigned a, b;
    edge(unsigned a, unsigned b): a(a), b(b) { }
    bool operator==(const edge &e) { return a == e.a && b == e.b || a == e.b && b == e.a; }
};

class edges {
public:
    vector<edge> v;
    void push_back(edge e) {
        if (find(v.begin(), v.end(), e) == v.end())
            v.push_back(e);
    }
    edge &operator[](size_t i) { return v[i]; }
    size_t size() { return v.size(); }
};

class Network {
public:
    vector<Node> nodes;
    Network(int node_cnt = 20);
    void build_channel(int node1_id, int node2_id);
    void build_random_channels();
    void start_dvmrp_and_routing();
    void transfer_packet(int src, int dest, int packet);
    void assign_group(int nodeid, int gid);
    void cancel_group(int nodeid, int gid);
private:
    default_random_engine e;
    uniform_int_distribution<unsigned> *u;
    edge create_edge(unsigned a);
};

Network::Network(int node_cnt) {
    u = new uniform_int_distribution<unsigned>(0, node_cnt - 1);
    for (int i = 0; i < node_cnt; ++i) {
        Node node(i);
        for (int j = 0; j < node_cnt; ++j) {
            RoutingTableEntry entry(j);
            node.insert_routing_table_entry(entry);
        }
        nodes.push_back(node);
    }
}

void Network::build_channel(int node1_id, int node2_id) {
    OVERFLOW_CHECK(node1_id, nodes)
    OVERFLOW_CHECK(node2_id, nodes)
    Node &node1 = nodes[node1_id];
    Node &node2 = nodes[node2_id];
    if (!node1.has_channel_for(&node2)) {
        node1.build_channel_for(&node2);
        node2.build_channel_for(&node1);
    }
}

void Network::build_random_channels() {

    edges es;
    uniform_int_distribution<unsigned> u(1, 3);
    unsigned edge_cnt = u(e);
    
    for (size_t i = 0; i < nodes.size(); ++i)
        for (size_t j = 0; j < edge_cnt; ++j)
            es.push_back(create_edge(i));

    for (size_t i = 0; i < es.size(); ++i)
        build_channel(es[i].a, es[i].b);
}

void Network::start_dvmrp_and_routing() {
    
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i].start_simplified_dvmrp();
        nodes[i].start_routing();
    }
}

void Network::transfer_packet(int src, int dest, int packet) {

    lock_guard<mutex> lg(mtx);
    OVERFLOW_CHECK(src, nodes)
    nodes[src].is_new_packet = true;
    nodes[src].packet = packet;
    nodes[src].src = src;
    nodes[src].dest = dest;
    nodes[src].from = src;
}

edge Network::create_edge(unsigned a) {

    unsigned b;

    do b = (*u)(e);
    while (b == a);

    return edge(a, b);
}

void Network::assign_group(int nodeid, int gid) {
    
    OVERFLOW_CHECK(nodeid, nodes)
    nodes[nodeid].join_group(gid);
}

void Network::cancel_group(int nodeid, int gid) {
    OVERFLOW_CHECK(nodeid, nodes)
    nodes[nodeid].leave_group(gid);
}

int main(void)
{
    Network net(10);

    net.build_random_channels();
    net.start_dvmrp_and_routing();

    net.assign_group(1, 18);
    net.assign_group(3, 18);
    net.assign_group(5, 18);
    net.assign_group(9, 18);
    net.assign_group(6, 18);

    int packet;
    unsigned src, dest;

    while (true) {
        cout << "Please enter the sending node: ";
        cin >> src;
        cout << "Please enter the receiving node: ";
        cin >> dest;
        cout << "Please enter the packet data: ";
        cin >> packet;
        net.transfer_packet(src, dest, packet);
    }

    return 0;
}
