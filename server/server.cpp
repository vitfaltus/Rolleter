#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

enum motor_states{
  stopped,
  backwards,
  forwards
};

enum device_states{
  idle,
  homing,
  light_movement,
  manual_movement
};

struct next_state {
    int device_id;
    int desired_angle;
    device_states dev_state;
    motor_states mot_state;
};

std::vector<char> serialize(const next_state& data){
    std::vector<char> buffer(sizeof(next_state));
    std::memcpy(buffer.data(), &data, sizeof(data));
    int* id_ptr = reinterpret_cast<int*>(buffer.data());
    *id_ptr = htonl(data.device_id);
    return buffer;
}

int log_event(const std::string& message){
    std::ofstream logFile("events.log", std::ios::app);

    if (!logFile){
        std::cerr << "Failed to open the file for writing." << std::endl;
        return 1;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time);
    logFile << std::put_time(local_time, "%Y-%m-%d %H:%M:%S") << " ---> ";
    logFile << message << std::endl;
    logFile.close();
    
    return 0;
}

void send_next_state(const next_state& to_send, , int client_soc){
    std::vector<char> buffer = serialize(to_send);
    int send_code = send(client_soc, buffer.data(), buffer.size(), 0);
    if (send_code < 0){
        log_event("next_state not sent");
    }
    else{
        log_event("next_state sent with data; id:" + std::to_string(to_send.id) + ", angle:" + std::to_string(to_send.desired_angle));
    }
}


int main(){
    int server_soc = socket(AF_INET, SOCK_STREAM, 0);
    if (server_soc < 0){
        log_event("Failed to create a socket");
        return 1;
    }
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(8080);

    if (bind(server_soc, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0){
        log_event("Failed to bind a socket");
        return 1;
    }

    if (listen(server_soc, 1) < 0){
        log_event("Failed to listen on socket");
        return 1;
    }

    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_soc = accept(server_soc, (struct sockaddr*)&client_addr, &client_len);
    log_event("Client connected");

    close(client_soc);
    close(server_soc);

    return 0;
}