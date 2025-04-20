#ifndef SOLVER_UTILS_H
#define SOLVER_UTILS_H

#include <vector>
#include <queue>
#include <string>

class Node {
    public:
        std::pair<int, int> state;
        Node* parent;
        std::string action;
    
        explicit Node(std::pair<int, int> state, Node* parent = nullptr, std::string action = "");
};

class StackFrontier {
    public:
        StackFrontier();
        void add(Node* node);
        bool contains_state(const std::pair<int, int> &state) const;
        bool empty() const;
        Node *remove();
    private:
        std::vector<Node*> frontier;
};

class QueueFrontier {
    public:
        QueueFrontier();
        void add(Node* node);
        bool contains_state(const std::pair<int, int> &state) const;
        bool empty() const;
        Node *remove();
    private:
        std::queue<Node*> frontier;
};

#endif