//
// Example how to adjust the speed during a mission.
//
// For PX4, make sure to adapt the param MPC_XY_VEL_MAX to the maximum velocity
// that you intend to fly at.
//
// Note that a speed of 0 is not accepted by PX4 but something like 0.01 m/s is.
//

#include <cctype>
#include <cstdint>
#include <future>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <thread>

using namespace mavsdk;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        }
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

Mission::MissionItem
make_mission_item(double latitude_deg, double longitude_deg, float relative_altitude_m)
{
    Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.is_fly_through = true;
    return new_item;
}

bool set_speed_to(MavlinkPassthrough& mp, float speed_m_s)
{
    auto command = MavlinkPassthrough::CommandLong{
        mp.get_target_sysid(),
        mp.get_target_compid(),
        MAV_CMD_DO_CHANGE_SPEED,
        1.0f, // Ground speed
        speed_m_s,
        -1.0f, // No change
        0.0f,
        0.0f,
        0.0f,
        0.0f};

    if (auto result = mp.send_command_long(command) != MavlinkPassthrough::Result::Success) {
        std::cerr << "Sending command failed: " << result << '\n';
        return false;
    }

    return true;
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system) {
        return 1;
    }

    // Instantiate plugins
    auto action = Action{system};
    auto mission = Mission{system};
    auto mavlink_passthrough = MavlinkPassthrough{system};

    // Upload simple square mission
    Mission::MissionPlan plan{};
    plan.mission_items.push_back(make_mission_item(47.3977507, 8.5456073, 20.0f));

    plan.mission_items.push_back(make_mission_item(47.39777622, 8.54679294, 20.0f));

    plan.mission_items.push_back(make_mission_item(47.39855329, 8.54685731, 20.0f));

    plan.mission_items.push_back(make_mission_item(47.39853877, 8.54555912, 20.0f));

    plan.mission_items.push_back(make_mission_item(47.39774717, 8.54561276, 20.0f));

    mission.set_return_to_launch_after_mission(true);

    if (auto result = mission.upload_mission(plan) != Mission::Result::Success) {
        std::cerr << "Mission upload failed: " << result << '\n';
        return 1;
    }

    // Arm and start mission
    if (auto result = action.arm() != Action::Result::Success) {
        std::cerr << "Arming failed: " << result << '\n';
        return 1;
    }

    // Start at slow speed
    if (!set_speed_to(mavlink_passthrough, 3.0f)) {
        return 1;
    }

    if (auto result = mission.start_mission() != Mission::Result::Success) {
        std::cerr << "Mission start failed: " << result << '\n';
        return 1;
    }

    // Wait a bit to let the takeoff prints go by
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Read user input to vary speed
    std::cout << "Enter speed in m/s, or q to exit, and hit enter\n";

    while (true) {
        std::string input;
        std::cin >> input;
        if (input == "\r") {
            continue;
        }
        if (input == "q") {
            continue;
        }

        try {
            float speed_m_s = std::stof(input);
            std::cout << "Set speed to " << input << " m/s\n";
            if (!set_speed_to(mavlink_passthrough, speed_m_s)) {
                return 1;
            }
        } catch (std::invalid_argument const& ex) {
            std::cout << "Could not parse: " << input << '\n';
        }
    }

    return 0;
}
