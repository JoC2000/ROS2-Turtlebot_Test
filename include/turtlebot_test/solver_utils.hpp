#pragma once

#include <vector>
#include <queue>
#include <string>
#include <algorithm>

class Agent {
    public:
        std::pair<int, int> state;
        Agent* parent;
        std::string action;
    
        explicit Agent(std::pair<int, int> state, Agent* parent = nullptr, std::string action = "");
};

class StackFrontier {
    public:
        StackFrontier();
        void add(Agent* node);
        bool contains_state(const std::pair<int, int> &state) const;
        bool empty() const;
        Agent *remove();
    private:
        std::vector<Agent*> frontier;
};

class QueueFrontier {
    public:
        QueueFrontier();
        void add(Agent* node);
        bool contains_state(const std::pair<int, int> &state) const;
        bool empty() const;
        Agent *remove();
    private:
        std::queue<Agent*> frontier;
};