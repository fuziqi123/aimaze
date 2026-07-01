#include "algorithm.h"

#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    // 曼哈顿距离作为启发式
    int heuristic(const Position& a, const Position& b)
    {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    // 检查坐标是否在地图范围内（这里我们无法精确知道地图边界，但基于已发现格子的范围做限制）
    // 我们只考虑在已发现格子及其邻居范围内的坐标
    bool isValid(const Position& pos, const std::map<Position, char>& grid)
    {
        return grid.find(pos) != grid.end();
    }

    // 获取四个方向的邻居
    std::vector<Position> getNeighbors(const Position& pos)
    {
        std::vector<Position> neighbors;
        neighbors.push_back({ pos.x - 1, pos.y });
        neighbors.push_back({ pos.x + 1, pos.y });
        neighbors.push_back({ pos.x, pos.y - 1 });
        neighbors.push_back({ pos.x, pos.y + 1 });
        return neighbors;
    }

    // A*节点
    struct Node
    {
        Position pos;
        int g;          // 实际代价
        int h;          // 启发式估计
        Node* parent;   // 父节点指针

        int f() const { return g + h; }

        // 比较器，用于优先队列（最小堆）
        struct Compare
        {
            bool operator()(const Node* a, const Node* b) const
            {
                return a->f() > b->f(); // 小顶堆，f越小优先级越高
            }
        };
    };

    // 从节点回溯路径
    std::vector<Position> reconstructPath(Node* node)
    {
        std::vector<Position> path;
        while (node)
        {
            path.push_back(node->pos);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // 判断目标是否已知（在memory中已发现）
    bool isTargetKnown(const Position& target, const std::map<Position, CellMemory>& memory)
    {
        auto it = memory.find(target);
        return it != memory.end() && it->second.discovered;
    }

    // 获取目标位置（优先终点，其次Boss，最后最近的未知格子）
    Position getTarget(const Position& start,
        const std::map<Position, CellMemory>& memory,
        const std::map<Position, char>& grid)
    {
        // 1. 查找终点（E）
        for (const auto& kv : memory)
        {
            if (kv.second.discovered && kv.second.type == 'E')
                return kv.first;
        }

        // 2. 查找Boss（B）
        for (const auto& kv : memory)
        {
            if (kv.second.discovered && kv.second.type == 'B')
                return kv.first;
        }

        // 3. 寻找最近的未知格子（即grid中存在但memory中未发现）
        Position target = start;
        int bestDist = std::numeric_limits<int>::max();
        bool found = false;

        for (const auto& kv : grid)
        {
            const Position& pos = kv.first;
            // 如果该格子不在memory中，或者未发现
            auto memIt = memory.find(pos);
            if (memIt == memory.end() || !memIt->second.discovered)
            {
                int dist = heuristic(start, pos);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    target = pos;
                    found = true;
                }
            }
        }

        // 如果找到了未知格子，返回它；否则返回起点（表示无需移动）
        return found ? target : start;
    }
}

Action Algorithm::AStar(const GameState& state, std::map<Position, CellMemory>& memory)
{
    const Position& start = state.player.position;

    // 构建已知地图（grid），包含memory中所有已发现格子及其类型
    std::map<Position, char> grid;
    for (const auto& kv : memory)
    {
        const Position& pos = kv.first;
        const CellMemory& cell = kv.second;
        if (cell.discovered)
        {
            grid[pos] = cell.type;
        }
    }

    // 扩展：将所有已发现格子的邻居也加入grid（视为可通行，类型设为' '），以便规划到未知区域
    std::map<Position, char> extendedGrid = grid;
    for (const auto& kv : grid)
    {
        const Position& pos = kv.first;
        for (const Position& neighbor : getNeighbors(pos))
        {
            if (extendedGrid.find(neighbor) == extendedGrid.end())
            {
                // 邻居未知，假设可通行
                extendedGrid[neighbor] = ' ';
            }
        }
    }

    // 确定目标
    Position target = getTarget(start, memory, extendedGrid);

    // 如果目标就是当前位置，无需移动
    if (target == start)
    {
        return Action{ ActionType::NONE };
    }

    // 运行A*
    std::priority_queue<Node*, std::vector<Node*>, Node::Compare> openList;
    std::map<Position, int> bestG;
    std::unordered_map<Position, Node*> nodeMap; // 用于快速查找节点

    Node* startNode = new Node{ start, 0, heuristic(start, target), nullptr };
    openList.push(startNode);
    bestG[start] = 0;
    nodeMap[start] = startNode;

    Node* resultNode = nullptr;
    std::map<Position, bool> closedSet;

    while (!openList.empty())
    {
        Node* current = openList.top();
        openList.pop();

        Position curPos = current->pos;
        if (closedSet.find(curPos) != closedSet.end())
            continue;
        closedSet[curPos] = true;

        if (curPos == target)
        {
            resultNode = current;
            break;
        }

        // 扩展四个方向
        for (const Position& neighbor : getNeighbors(curPos))
        {
            // 检查邻居是否在扩展网格中且不是墙
            auto it = extendedGrid.find(neighbor);
            if (it == extendedGrid.end() || it->second == '#')
                continue;

            // 如果邻居已经在closed中，跳过
            if (closedSet.find(neighbor) != closedSet.end())
                continue;

            int newG = current->g + 1;
            auto bestIt = bestG.find(neighbor);
            if (bestIt == bestG.end() || newG < bestIt->second)
            {
                bestG[neighbor] = newG;
                Node* neighborNode = new Node{ neighbor, newG, heuristic(neighbor, target), current };
                openList.push(neighborNode);
                nodeMap[neighbor] = neighborNode;
            }
        }
    }

    Action result;
    result.type = ActionType::NONE;

    if (resultNode)
    {
        // 回溯路径
        std::vector<Position> path = reconstructPath(resultNode);
        // 如果路径长度大于1，则返回第一步方向
        if (path.size() > 1)
        {
            const Position& next = path[1];
            if (next.x < start.x)
                result.direction = Direction::UP;
            else if (next.x > start.x)
                result.direction = Direction::DOWN;
            else if (next.y < start.y)
                result.direction = Direction::LEFT;
            else if (next.y > start.y)
                result.direction = Direction::RIGHT;
            result.type = ActionType::MOVE;
        }
        // 否则目标可达但已在起点（不应发生，因为前面已处理）
    }

    // 清理内存
    for (auto& kv : nodeMap)
    {
        delete kv.second;
    }

    return result;
}
