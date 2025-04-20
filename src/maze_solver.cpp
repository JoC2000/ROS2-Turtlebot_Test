#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>
#include <queue>

class Node {
    public:
        std::pair<int, int> state;
        Node* parent;
        std::string action;

        explicit Node(std::pair<int, int> state, Node* parent = nullptr, std::string action = "") : state(state), parent(parent), action(action) {}
};

class StackFrontier {
    private:
        std::vector<Node*> frontier;
    public:
        StackFrontier() = default;

        void add(Node* node) {
            frontier.push_back(node);
        }

        bool contains_state(const std::pair<int, int> &state) const {
            for (const auto &node : frontier) {
                if (node->state == state) {
                    return true;
                }
            }
            return false;
        }

        bool empty() const {
            return frontier.empty();
        }

        Node *remove() {
            if (empty()) {
                throw std::runtime_error("Empty frontier");
            }
            Node *node = frontier.back();
            frontier.pop_back(); 
            return node;
        }
};

class QueueFrontier {
    private:
        std::queue<Node*> frontier;
    public:
        QueueFrontier() = default;

        void add(Node* node) {
            frontier.push(node);
        }

        bool contains_state(const std::pair<int, int> &state) const {
            std::queue<Node*> temp = frontier;
            while (!temp.empty()) {
                Node *node = temp.front();
                if (node->state == state) {
                    return true;
                }
                temp.pop();
            }
            return false;
        }

        bool empty() const {
            return frontier.empty();
        }

        Node *remove() {
            if (empty()) {
                throw std::runtime_error("Empty frontier");
            }
            Node *node = frontier.front();
            frontier.pop();
            return node;
        }
};