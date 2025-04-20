#include <turtlebot_test/solver_utils.hpp>
#include <stdexcept>

Node::Node(std::pair<int, int> state, Node* parent, std::string action)
    : state(state), parent(parent), action(std::move(action)) {}

StackFrontier::StackFrontier() = default;

void StackFrontier::add(Node* node) {
    frontier.push_back(node);
}

bool StackFrontier::contains_state(const std::pair<int, int> &state) const {
    for (const auto &node : frontier) {
        if (node->state == state) {
            return true;
        }
    }
    return false;
}

bool StackFrontier::empty() const {
    return frontier.empty();
}

Node* StackFrontier::remove() {
    if (empty()) {
        throw std::runtime_error("Empty stack");
    }
    Node* node = frontier.back();
    frontier.pop_back();
    return node;
}

QueueFrontier::QueueFrontier() = default;

void QueueFrontier::add(Node* node) {
    frontier.push(node);
}

bool QueueFrontier::contains_state(const std::pair<int, int> &state) const {
    std::queue<Node*> temp = frontier;
    while (!temp.empty()) {
        if (temp.front()->state == state) {
            return true;
        }
        temp.pop();
    }
    return false;
}

bool QueueFrontier::empty() const {
    return frontier.empty();
}

Node*QueueFrontier::remove() {
    if (empty()) {
        throw std::runtime_error("Empty queue");
    }
    Node* node = frontier.front();
    frontier.pop();
    return node;
}