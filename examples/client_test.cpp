#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <unistd.h>     // close()
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;

const char* SERVER_IP = "127.0.0.1";
const int SERVER_PORT = 12345;
const vector<string> labels = {"x", "y", "z", "roll", "pitch", "yaw"};

void receive_loop(int sockfd) {
    char buffer[1024];
    while (true) {
        ssize_t len = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
        if (len <= 0) break;
        buffer[len] = '\0';

        string data(buffer);
        if (data == "s") {
            cout << "[STOP] Received stop signal from server." << endl;
            continue;
        }

        try {
            json received = json::parse(data);
            cout << "[SERVER] Feedback:" << endl;
            for (const auto& label : labels) {
                cout << "  " << label << ": " << received[label] << endl;
            }
        } catch (...) {
            cout << "[WARN] Invalid JSON received: " << data << endl;
        }
    }
}

int main() {
    // 소켓 생성
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        cerr << "Socket creation failed" << endl;
        return 1;
    }

    // 서버 주소 설정
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);

    // 서버 연결
    if (connect(sockfd, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        cerr << "Connection failed" << endl;
        return 1;
    }

    cout << "[CLIENT] Connected to server." << endl;

    // 수신 스레드 실행
    thread recv_thread(receive_loop, sockfd);

    while (true) {
        cout << "Enter 6 values (x y z roll pitch yaw): ";
        string line;
        getline(cin, line);
        istringstream iss(line);
        vector<float> values;
        float val;
        while (iss >> val) values.push_back(val);

        if (values.size() != 6) {
            cout << "[ERROR] Please enter exactly 6 numbers." << endl;
            continue;
        }

        json msg;
        for (size_t i = 0; i < labels.size(); ++i) {
            msg[labels[i]] = values[i];
        }

        string out = msg.dump();
        send(sockfd, out.c_str(), out.length(), 0);
    }

    recv_thread.join();
    close(sockfd);
    return 0;
}
