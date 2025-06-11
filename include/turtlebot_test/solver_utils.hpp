#pragma once

#include <vector>
#include <queue>
#include <string>
#include <algorithm>

/**
 * @class Agent
 * @brief Represents the Node for the search algorithm, stores state, parent node and action.
*/
class Agent {
public:
    /**
     * @brief Constructor for Agent object.
     * @param state A pair of integers that represents the state of the agent in map cell coordinates x and y.
     * @param parent Pointer to the parent Node, default is `nullptr`.
     * @param action A string that represents the action taken to reach this node state.
    */
    explicit Agent(
        std::pair<int, int> state,
        Agent* parent = nullptr,    
        std::string action = ""
    );
    
    std::pair<int, int> state;
    Agent* parent;
    std::string action;
};

/**
 * @class StackFrontier
 * @brief Represents the Deep First Search (DFS) frontier approach. 
*/
class StackFrontier {
public:
    /** 
     * @brief Constructor for StackFrontier object.
    */
    StackFrontier();

    /**
     * @brief Adds the Agent pointer to the frontier.
     * @param node An Agent type pointer.
    */
    void add(Agent* node);

    /**
     * @brief Check for a specific state inside the frontier.
     * @param state A pair of integers that represents the state of the agent in map cell coordinates x and y.
     * @return `True` if the state is found in the frontier. `False` otherwise.
    */
    bool contains_state(const std::pair<int, int> &state) const;

    /**
     * @brief Checks if frontier is empty.
     * @return `True` if frontier is empty. `False` otherwise. 
    */
    bool empty() const;

    /**
     * @brief Removes Agent pointer from the frontier.
     * @return Removed Agent pointer. 
    */
    Agent *remove();
private:
    std::vector<Agent*> frontier;
};

/**
 * @class StackFrontier
 * @brief Represents the Breadth First Search (BFS) frontier approach.
*/
class QueueFrontier {
public:
    /** 
     * @brief Constructor for QueueFrontier object.
    */
    QueueFrontier();

    /**
     * @brief Adds the Agent pointer to the frontier.
     * @param node An Agent type pointer.
    */
    void add(Agent* node);

    /**
     * @brief Check for a specific state inside the frontier.
     * @param state A pair of integers that represents the state of the agent in map cell coordinates x and y.
     * @return `True` if the state is found in the frontier. `False` otherwise.
    */
    bool contains_state(const std::pair<int, int> &state) const;

    /**
     * @brief Checks if frontier is empty.
     * @return `True` if frontier is empty. `False` otherwise. 
    */
    bool empty() const;

    /**
     * @brief Removes Agent pointer from the frontier.
     * @return Removed Agent pointer. 
    */
    Agent *remove();
private:
    std::queue<Agent*> frontier;
};