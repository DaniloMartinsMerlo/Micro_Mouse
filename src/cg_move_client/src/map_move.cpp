#include <rclcpp/rclcpp.hpp>
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>
#include <set>
#include <stack>
#include <queue>
#include <cmath>
#include <algorithm>
using std::string;

std::vector<std::vector<string>> grid(29, std::vector<string>(29, "x")); 
int target_id;
int initial_x = 1;
int initial_y = 1;
int actual_x = initial_x;
int actual_y = initial_y;
std::set<std::pair<int, int>> visitados;
std::stack<std::pair<int, int>> pilha_dfs;

int start_id_astar;
int target_id_astar;
using pii = std::pair<int,int>;
/*

    CLASSES PARA COMUNICAÇÃO COM O ROS

*/

// Sensores
class Sensors : public rclcpp::Node{
    public:
        Sensors() : Node("robot_sensors"){
            auto callback =
                [this](const cg_interfaces::msg::RobotSensors::SharedPtr msg) -> void {
                    last_msg_ = *msg;
                    RCLCPP_INFO(this->get_logger(),
                        "up=%s down=%s left=%s right=%s",
                        msg->up.c_str(), msg->down.c_str(), msg->left.c_str(), msg->right.c_str());
                };

            subscription_ = this->create_subscription<cg_interfaces::msg::RobotSensors>("/culling_games/robot_sensors", 10, callback);
        }
        
        cg_interfaces::msg::RobotSensors get_last_msg(){
            return last_msg_;
        }
    
    private:
        rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr subscription_;
        cg_interfaces::msg::RobotSensors last_msg_;
};

// Reset Client
class ResetClient : public rclcpp::Node{
    public:
        ResetClient() : Node("reset_client"){
            client_ = this->create_client<cg_interfaces::srv::Reset>("/reset");
        }
        
        void send_reset(bool is_random = false){            
            auto request = std::make_shared<cg_interfaces::srv::Reset::Request>();
            request->is_random = is_random;
            
            RCLCPP_INFO(this->get_logger(), "Enviando comando de reset");            
            auto result = client_->async_send_request(request);
        }
        
    private:
        rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr client_;
};

// Movimento
class Move : public rclcpp::Node{
    public:
        Move() : Node("move_client"){
            client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        }

        void send_command(const std::string &comando){
            auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
            request->direction = comando;

            RCLCPP_INFO(this->get_logger(), "Enviando comando: %s", request->direction.c_str());

            client_->async_send_request(request);
        }

    private:
        rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
};

/*

    MAPEAMENTO

*/

bool fullGrid(const std::vector<std::vector<string>>& grid){
    for (int i = 0; i < grid.size(); i++){
        for (int j = 0; j < grid[i].size(); j++){
            if (grid[i][j] == "x"){
                return false;
            }
        }
    }
    return true;
}

void update_cell(std::vector<std::vector<string>>& grid, int y, int x, const string& value) {
    if (y >= 0 && y < grid.size() && x >= 0 && x < grid[0].size()) {
        if (grid[y][x] == "x") {
            grid[y][x] = value;
        }
    }
}

void inputGrid(std::vector<std::vector<string>>& grid, cg_interfaces::msg::RobotSensors& msg){
    if (grid[actual_y][actual_x] == "x") {
        grid[actual_y][actual_x] = "f";
    }
    update_cell(grid, actual_y - 1, actual_x - 1, msg.up_left);
    update_cell(grid, actual_y - 1, actual_x, msg.up);
    update_cell(grid, actual_y - 1, actual_x + 1, msg.up_right);

    update_cell(grid, actual_y, actual_x - 1, msg.left);
    update_cell(grid, actual_y, actual_x + 1, msg.right);

    update_cell(grid, actual_y + 1, actual_x - 1, msg.down_left);
    update_cell(grid, actual_y + 1, actual_x, msg.down);
    update_cell(grid, actual_y + 1, actual_x + 1, msg.down_right);
}

void print_grid(){
    std::cout << "\n=== MAPA FINAL DO LABIRINTO ===\n\n";
    
    for (auto &row : grid){
        for (auto c : row) {
            std::cout << c << " ";
        }
        std::cout << "\n";
    }
}

/*

    MOVIMENTAÇÃO

*/

std::vector<std::pair<int, int>> get_unvisited_neighbors(const cg_interfaces::msg::RobotSensors& msg, int y, int x){
    std::vector<std::pair<int, int>> neighbors;
    
    if (msg.up == "f" && visitados.find({y - 1, x}) == visitados.end()) {
        neighbors.push_back({y - 1, x});
    }
    if (msg.right == "f" && visitados.find({y, x + 1}) == visitados.end()) {
        neighbors.push_back({y, x + 1});
    }
    if (msg.down == "f" && visitados.find({y + 1, x}) == visitados.end()) {
        neighbors.push_back({y + 1, x});
    }
    if (msg.left == "f" && visitados.find({y, x - 1}) == visitados.end()) {
        neighbors.push_back({y, x - 1});
    }
    
    return neighbors;
}

bool move_to_position(Move& move_client, int target_y, int target_x, rclcpp::executors::MultiThreadedExecutor& executor){
    string direction = "";
    
    if (target_y < actual_y) {
        direction = "up";
        actual_y--;
    } else if (target_y > actual_y) {
        direction = "down";
        actual_y++;
    } else if (target_x < actual_x) {
        direction = "left";
        actual_x--;
    } else if (target_x > actual_x) {
        direction = "right";
        actual_x++;
    } else {
        return false; 
    }
    
    move_client.send_command(direction);
    
    usleep(300000);
    executor.spin_some();
    usleep(300000);
    executor.spin_some();
    
    return true;
}

void dfs_maze_exploration(Move& move_client, rclcpp::executors::MultiThreadedExecutor& executor, std::shared_ptr<Sensors> sensors){
    visitados.insert({actual_y, actual_x});
    pilha_dfs.push({actual_y, actual_x});
    
    while (!fullGrid(grid)) {
        executor.spin_some();
        usleep(100000);
        auto msg = sensors->get_last_msg();
        
        if (msg.up == "" && msg.down == "" && msg.left == "" && msg.right == "") {
            RCLCPP_INFO(sensors->get_logger(), "Aguardando mensagem do sensor...");
            continue;
        }
        
        inputGrid(grid, msg);
        
        if (fullGrid(grid)) {
            RCLCPP_INFO(sensors->get_logger(), 
                "Mapeamento completo! Todas as células foram exploradas.");
            break;
        }
        
        auto neighbors = get_unvisited_neighbors(msg, actual_y, actual_x);
        
        if (!neighbors.empty()) {
            auto next = neighbors[0];
            
            RCLCPP_INFO(sensors->get_logger(), 
                "Posição atual: (%d, %d) -> Movendo para: (%d, %d) | Visitados: %zu", 
                actual_y, actual_x, next.first, next.second, visitados.size());
            
            move_to_position(move_client, next.first, next.second, executor);
            
            visitados.insert({actual_y, actual_x});
            pilha_dfs.push({actual_y, actual_x});
            
        } else {
            if (pilha_dfs.size() > 1) {
                pilha_dfs.pop(); 
                
                if (!pilha_dfs.empty()) {
                    auto target = pilha_dfs.top();
                    
                    RCLCPP_INFO(sensors->get_logger(), 
                        "Backtracking de (%d, %d) para (%d, %d)", 
                        actual_y, actual_x, target.first, target.second);
                    
                    move_to_position(move_client, target.first, target.second, executor);
                }
            }
        }
    }
}

/*

    ALGORITMO A* 

*/

std::vector<std::vector<int>> build_adj_matrix(){
    const int N = 29 * 29;
    std::vector<std::vector<int>> mat(N, std::vector<int>(N, 0));

    int di[4] = {-1, 1, 0, 0};
    int dj[4] = {0, 0, -1, 1};

    for (int i = 0; i < 29; i++){
        for (int j = 0; j < 29; j++){
            string c = grid[i][j];
            int id = i * 29 + j;

            if (c == "b")
                continue;

            if (c == "r")
                start_id_astar = id;

            if (c == "t")
                target_id_astar = id;

            for (int k = 0; k < 4; k++){
                int ni = i + di[k];
                int nj = j + dj[k];

                if (ni < 0 || ni >= 29 || nj < 0 || nj >= 29)
                    continue;

                if (grid[ni][nj] == "b")
                    continue;

                int id2 = ni * 29 + nj;

                mat[id][id2] = 1;
                mat[id2][id] = 1;
            }
        }
    }

    return mat;
}

int heuristic(int id){
    int x = id / 29;
    int y = id % 29;

    int tx = target_id_astar / 29;
    int ty = target_id_astar % 29;

    return std::abs(x - tx) + std::abs(y - ty);
}

std::vector<int> a_star(const std::vector<std::vector<int>>& adj) {
    const int N = 29 * 29;

    std::vector<int> g_score(N, 1e9);
    std::vector<int> f_score(N, 1e9);
    std::vector<int> came_from(N, -1);
    
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> open_set;

    g_score[start_id_astar] = 0;
    f_score[start_id_astar] = heuristic(start_id_astar);

    std::vector<bool> visited(N, false);

    open_set.push({f_score[start_id_astar], start_id_astar});
    while (!open_set.empty()) {
        int current = open_set.top().second;
        open_set.pop();

        if (visited[current]) continue;
        visited[current] = true;

        if (current == target_id_astar) break;

        for (int v = 0; v < N; v++) {
            if (adj[current][v] == 1) {
                int tentative_g = g_score[current] + 1;

                if (tentative_g < g_score[v]) {
                    came_from[v] = current;
                    g_score[v] = tentative_g;
                    f_score[v] = tentative_g + heuristic(v);

                    open_set.push({f_score[v], v});
                }
            }
        }
    }

    return came_from;  
}

std::vector<int> reconstruct_path(int start, int goal, const std::vector<int>& came_from) {
    std::vector<int> path;

    int current = goal;
    while (current != -1) {
        path.push_back(current);
        if (current == start)
            break;
        current = came_from[current];
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::string> path_to_moves(const std::vector<int>& path) {
    std::vector<std::string> moves;

    for (int k = 0; k < (int)path.size() - 1; k++) {
        int id1 = path[k];
        int id2 = path[k+1];

        int i1 = id1 / 29; 
        int j1 = id1 % 29;

        int i2 = id2 / 29;
        int j2 = id2 % 29;

        if (i2 == i1 - 1 && j2 == j1)
            moves.push_back("up");
        else if (i2 == i1 + 1 && j2 == j1)
            moves.push_back("down");
        else if (i2 == i1 && j2 == j1 - 1)
            moves.push_back("left");
        else if (i2 == i1 && j2 == j1 + 1)
            moves.push_back("right");
    }

    return moves;
}

void execute_astar_path(Move& move_client, rclcpp::executors::MultiThreadedExecutor& executor, std::shared_ptr<Sensors> sensors){
    std::cout << "\n=== EXECUTANDO A* ===\n";
    
    std::vector<std::vector<int>> adj = build_adj_matrix();
    
    std::cout << "Start ID: " << start_id_astar << " (y=" << start_id_astar/29 << ", x=" << start_id_astar%29 << ")\n";
    std::cout << "Target ID: " << target_id_astar << " (y=" << target_id_astar/29 << ", x=" << target_id_astar%29 << ")\n";
    std::cout << "Heurística para o start: " << heuristic(start_id_astar) << "\n";
    
    // Executar A*
    auto came = a_star(adj);
    auto path = reconstruct_path(start_id_astar, target_id_astar, came);
    
    std::cout << "\nCaminho encontrado (IDs):\n";
    for (int id : path)
        std::cout << id << " ";
    std::cout << "\n";
    
    auto moves = path_to_moves(path);
    
    std::cout << "\nMovimentos:\n";
    for (auto &m : moves)
        std::cout << "\"" << m << "\", ";
    std::cout << "\n";
    
    std::cout << "\n=== INICIANDO NAVEGAÇÃO PARA O TARGET ===\n";
    
    for (size_t i = 0; i < moves.size(); i++) {
        RCLCPP_INFO(sensors->get_logger(), "Movimento %zu/%zu: %s", i+1, moves.size(), moves[i].c_str());
        move_client.send_command(moves[i]);
        
        usleep(50000);
        executor.spin_some();
    }    
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto sensors = std::make_shared<Sensors>();
    auto move = std::make_shared<Move>();
    auto reset = std::make_shared<ResetClient>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensors);
    executor.add_node(move);
    executor.add_node(reset);

    usleep(200000);
    executor.spin_some();
    usleep(200000);
 
    grid[actual_y][actual_x] = "r";
    
    dfs_maze_exploration(*move, executor, sensors);

    print_grid();
    RCLCPP_INFO(sensors->get_logger(), "Mapeamento concluído!");

    reset->send_reset(false);
    
    usleep(1000000);
    executor.spin_some();
    
    actual_x = initial_x;
    actual_y = initial_y;
    
    execute_astar_path(*move, executor, sensors);
        
    rclcpp::shutdown();
    return 0;
}