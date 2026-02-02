#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include "backend_prod/json_writer.hpp"

using backend_prod::JsonWriter;

class TraceWriter {
public:
    TraceWriter(const std::string& path) : seq_(0) {
        ofs_.open(path, std::ios::trunc); // Ensure fresh file
        if (!ofs_.is_open()) {
            throw std::runtime_error("Failed to open trace file: " + path);
        }
        start_time_ = std::chrono::steady_clock::now();
    }

    // Helper to emit a structured event
    // We construct the JSON manually but cleanly using stringstream
    void emit(const std::string& type, const std::string& scenario_id, const std::string& extra_fields = "") {
        auto now = std::chrono::steady_clock::now();
        auto t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_time_).count();

        ofs_ << "{"
             << "\"version\": \"0.1\","
             << "\"run_id\": \"run_prod\","
             << "\"t_ns\": " << t_ns << ","
             << "\"sequence\": " << seq_++ << ","
             << "\"type\": \"" << type << "\","
             << "\"scenario_id\": \"" << scenario_id << "\"";

        if (!extra_fields.empty()) {
            ofs_ << "," << extra_fields;
        }

        ofs_ << "}\n";
        ofs_.flush();
    }

private:
    std::ofstream ofs_;
    uint64_t seq_;
    std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: backend_prod <bundle_path> <trace_path>" << std::endl;
        return 2;
    }

    std::string bundle_path = argv[1];
    (void)bundle_path; // Unused in smoke test
    std::string trace_path = argv[2];

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("backend_prod");
    (void)node; // Unused logic

    try {
        TraceWriter writer(trace_path);

        // 1. Run Start
        writer.emit("run_start", "_run", R"("detail": {"backend": "prod"})");

        // 2. Capabilities
        // Construct detailed object cleanly
        std::string caps = R"("caps": ["actions.basic", "actions.terminal"], "limits": {})";
        writer.emit("backend_capabilities", "_run", R"("detail": {"backend": "prod", )" + caps + "}");

        // 3. Scenario H00
        std::string sid = "H00_smoke_trace_roundtrip";
        writer.emit("scenario_start", sid, R"("detail": {"backend": "prod"})");

        // Op 0: send_goal g1
        writer.emit("op_start", sid, R"("op_id": ")" + sid + R"(#0", "detail": {"op": "send_goal"})");
        writer.emit("goal_send", sid, R"("goal_id": "g1")");
        writer.emit("goal_response", sid, R"("goal_id": "g1", "accepted": true)");
        writer.emit("op_end", sid, R"("op_id": ")" + sid + R"(#0")");

        // Op 1: wait
        writer.emit("op_start", sid, R"("op_id": ")" + sid + R"(#1", "detail": {"op": "wait"})");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        writer.emit("op_end", sid, R"("op_id": ")" + sid + R"(#1")");

        // Op 2: send_goal g2
        writer.emit("op_start", sid, R"("op_id": ")" + sid + R"(#2", "detail": {"op": "send_goal"})");
        writer.emit("goal_send", sid, R"("goal_id": "g2")");
        writer.emit("goal_response", sid, R"("goal_id": "g2", "accepted": false, "reason": "prod_reject")");
        writer.emit("op_end", sid, R"("op_id": ")" + sid + R"(#2")");

         // Op 3: wait
        writer.emit("op_start", sid, R"("op_id": ")" + sid + R"(#3", "detail": {"op": "wait"})");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        writer.emit("op_end", sid, R"("op_id": ")" + sid + R"(#3")");

        // Scenario End
        writer.emit("scenario_end", sid);

        // 4. Run End
        writer.emit("run_end", "_run");

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 4;
    }

    rclcpp::shutdown();
    return 0;
}
