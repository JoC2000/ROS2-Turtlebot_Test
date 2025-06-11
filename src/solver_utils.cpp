#include <turtlebot_test/solver_utils.hpp>
#include <stdexcept>

//  Agent Implementation -----------------------------------------------------------
Agent::Agent(
    std::pair<int, int> state,
    Agent* parent,
    std::string action
)   : state(state)
    , parent(parent)
    , action(std::move(action)) {}

// StackFrontier Implementation ----------------------------------------------------
StackFrontier::StackFrontier() = default;

void StackFrontier::add(Agent* node) {
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

Agent* StackFrontier::remove() {
    if (empty()) {
        throw std::runtime_error("Empty stack");
    }
    Agent* node = frontier.back();
    frontier.pop_back();
    return node;
}


// QueueFrontier Implementation ----------------------------------------------------
QueueFrontier::QueueFrontier() = default;

void QueueFrontier::add(Agent* node) {
    frontier.push(node);
}

bool QueueFrontier::contains_state(const std::pair<int, int> &state) const {
    std::queue<Agent*> temp = frontier;
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

Agent*QueueFrontier::remove() {
    if (empty()) {
        throw std::runtime_error("Empty queue");
    }
    Agent* node = frontier.front();
    frontier.pop();
    return node;
}