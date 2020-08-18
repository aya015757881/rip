#include <iostream>
#include <sstream>
#include <vector>
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

mutex rip_mtx;
mutex routing_mtx;

class RoutingTableEntry {
public:
    int dest_id;
    int next_hop_id;
    int cost;
    RoutingTableEntry(int dest);
    void print();
};

RoutingTableEntry::RoutingTableEntry(int dest): dest_id(dest), next_hop_id(-1), cost(16) { }

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
    bool is_new_packet = false;
    int packet = 0;
    unsigned dest = 0;
    vector<Node*> neighbors;
    vector<RoutingTableEntry> routing_table;

    Node(): Node(-1) { }
    Node(int id): id(id) { }
    
    void start_routing();
    void start_simplified_rip();
    void run_routing();
    void run_simplified_rip();
    void insert_routing_table_entry(RoutingTableEntry &entry);
    void print_routing_table();
    bool has_channel_for(Node *node);
    void build_channel_for(Node *node);
};

void Node::start_routing() {
    thread t(&Node::run_routing, this);
    t.detach();
}

void Node::start_simplified_rip() {
    thread t(&Node::run_simplified_rip, this);
    t.detach();
}

void Node::run_routing() {

    while (true) {

        lock_guard<mutex> lg(routing_mtx);

        if (is_new_packet) {
            stringstream ss;
            int next_hop = routing_table[dest].next_hop_id;
            ss << "\n\nnode:\t" << id << "\npacket:\t" << packet
                << "\ndest:\t" << dest << "\nstate: ";
            if (next_hop == -1)
                ss << "dropped\n";
            else if (next_hop == id)
                ss << "received\n";
            else
                for (size_t i = 0; i < neighbors.size(); ++i)
                    if (neighbors[i]->id == next_hop) {
                        neighbors[i]->is_new_packet = true;
                        neighbors[i]->packet = packet;
                        neighbors[i]->dest = dest;
                        ss << "delivered to next hop node: " << next_hop << "\n";
                        break;
                    }        
            is_new_packet = false;
            cout << ss.str() << endl;
        }
    }
}

void Node::run_simplified_rip() {

    for (int i = 0; i < routing_table.size(); i = i < routing_table.size() - 1 ? i + 1 : 0) {

        auto &this_entry = routing_table[i];
        lock_guard<mutex> lg(rip_mtx);

        if (this_entry.dest_id == id) {
            this_entry.next_hop_id = id;
            this_entry.cost = 0;
            continue;
        }

        bool is_neighbor = false;
        for (int k = 0; k < neighbors.size(); ++k)
            if (this_entry.dest_id == neighbors[k]->id) {
                is_neighbor = true;
                break;
            }
        if (is_neighbor) {
            this_entry.next_hop_id = this_entry.dest_id;
            this_entry.cost = 1;
            continue;
        }

        for (int j = 0; j < neighbors.size(); ++j) {
            auto &nbr_entry = neighbors[j]->routing_table[i];

            if (nbr_entry.next_hop_id == id &&
                this_entry.next_hop_id == neighbors[j]->id) {
                this_entry.next_hop_id = -1;
                this_entry.cost = 16;
                continue;
            }

            if (this_entry.next_hop_id == neighbors[j]->id) {
                if (nbr_entry.next_hop_id == -1) {
                    this_entry.next_hop_id = -1;
                    this_entry.cost = 16;
                    continue;
                }
                this_entry.cost = min(nbr_entry.cost + 1, 16);
                if (this_entry.cost == 16)
                    this_entry.next_hop_id = -1;
                continue;
            }

            if (nbr_entry.next_hop_id != -1 &&
                nbr_entry.cost + 1 < 16 &&
                nbr_entry.cost + 1 < this_entry.cost) {
                this_entry.next_hop_id = neighbors[j]->id;
                this_entry.cost = nbr_entry.cost + 1;
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
    void build_channel(unsigned node1_id, unsigned node2_id);
    void build_random_channels();
    void start_rip_and_routing();
    void transfer_packet(unsigned src, unsigned dest, int packet);
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

void Network::build_channel(unsigned node1_id, unsigned node2_id) {
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

void Network::start_rip_and_routing() {
    
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i].start_simplified_rip();
        nodes[i].start_routing();
    }
}

void Network::transfer_packet(unsigned src, unsigned dest, int packet) {

    lock_guard<mutex> lg(routing_mtx);
    nodes[src].is_new_packet = true;
    nodes[src].packet = packet;
    nodes[src].dest = dest;
}

edge Network::create_edge(unsigned a) {

    unsigned b;

    do b = (*u)(e);
    while (b == a);

    return edge(a, b);
}

int main(void)
{
    Network net(10);

    net.build_random_channels();
    net.start_rip_and_routing();

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
