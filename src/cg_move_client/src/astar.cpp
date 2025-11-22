#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <cmath>
#include <algorithm>
#include <chrono>

std::vector<std::vector<char>> grid; 
int start_id;
int target_id;
using pii = std::pair<int,int>;

class Map : public rclcpp::Node {
public:
    Map() : Node("get_map_client") {
        client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
    }

    std::shared_ptr<cg_interfaces::srv::GetMap::Response> get_map() {
        RCLCPP_INFO(this->get_logger(), "Aguardando serviço /get_map...");
        
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Serviço não disponível, aguardando...");
        }

        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto future = client_->async_send_request(req);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Resposta recebida com sucesso!");
            return future.get();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Falha ao chamar serviço /get_map");
            return nullptr;
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_;
};

void fill_grid_from_service(const std::shared_ptr<cg_interfaces::srv::GetMap::Response> res) {
    int linhas = res->occupancy_grid_shape[0];
    int colunas = res->occupancy_grid_shape[1];

    std::cout << "Dimensões do grid: " << linhas << "x" << colunas << "\n";
    std::cout << "Total de células: " << res->occupancy_grid_flattened.size() << "\n";

    grid.assign(linhas, std::vector<char>(colunas));

    for (int i = 0; i < linhas; i++) {
        for (int j = 0; j < colunas; j++) {
            std::string cell = res->occupancy_grid_flattened[i * colunas + j];
            grid[i][j] = cell[0];
        }
    }
}

void print_grid(){
    std::cout << "\nGRID:\n\n";
    for (auto &row : grid){
        for (char c : row) std::cout << c << " ";
        std::cout << "\n";
    }
}

std::vector<std::vector<int>> build_adj_matrix(){
    const int N = 29 * 29;
    std::vector<std::vector<int>> mat(N, std::vector<int>(N, 0));

    int di[4] = {-1, 1, 0, 0};
    int dj[4] = {0, 0, -1, 1};

    for (int i = 0; i < 29; i++){
        for (int j = 0; j < 29; j++){

            char c = grid[i][j];
            int id = i * 29 + j;

            if (c == 'b')
                continue;

            if (c == 'r')
                start_id = id;

            if (c == 't')
                target_id = id;

            for (int k = 0; k < 4; k++){
                int ni = i + di[k];
                int nj = j + dj[k];

                if (ni < 0 || ni >= 29 || nj < 0 || nj >= 29)
                    continue;

                if (grid[ni][nj] == 'b')
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

    int tx = target_id / 29;
    int ty = target_id % 29;

    return std::abs(x - tx) + std::abs(y - ty);
}

std::vector<int> a_star(const std::vector<std::vector<int>>& adj) {

    const int N = 29 * 29;

    std::vector<int> g_score(N, 1e9);
    std::vector<int> f_score(N, 1e9);
    std::vector<int> came_from(N, -1);
    
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> open_set;

    g_score[start_id] = 0;
    f_score[start_id] = heuristic(start_id);

    std::vector<bool> visited(N, false);

    open_set.push({f_score[start_id], start_id});
    while (!open_set.empty()) {

        int current = open_set.top().second;
        open_set.pop();

        if (visited[current]) continue;
        visited[current] = true;

        if (current == target_id) break;

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

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Map>();

    std::cout << "=== Iniciando cliente do serviço GetMap ===\n\n";

    auto response = node->get_map();

    std::cout << "\n=== Processando mapa recebido ===\n";
    fill_grid_from_service(response);
    print_grid();

    std::vector<std::vector<int>> adj = build_adj_matrix();
    std::cout << "\nStart ID: " << start_id << " (posição: " << start_id/29 << "," << start_id%29 << ")\n";
    std::cout << "Target ID: " << target_id << " (posição: " << target_id/29 << "," << target_id%29 << ")\n";

    std::cout << "\n=== Executando A* ===\n";
    auto came = a_star(adj);
    auto path = reconstruct_path(start_id, target_id, came);
    auto moves = path_to_moves(path);
    
    std::cout << "\nMovimentos:\n";
    for (auto &m : moves)
        std::cout << "\"" << m << "\", ";
    std::cout << "\n";

    rclcpp::shutdown();
    return 0;
}