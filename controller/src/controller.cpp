// controller.cpp
// Functions for controlling the robot

#include "controller.hpp"
#include "socket.hpp"

//librairies protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <protos/api_v2_rtos.pb.h>

google::protobuf::Any asAny(google::protobuf::Message& msg)
{
    google::protobuf::Any any;
    any.PackFrom(msg);
    return any;
}

flr_api::v2::rtos::SystemState Command::receive_system_state(Socket& socket)
{
	// Created to avoid repeating this part of code in every other functions to receive state messages and parse it
	// 1. Reception of protobuf message
    std::string msg;
    try {
        socket.receive(msg, 1024, 2);

    } catch(const std::exception& e) {

        throw std::runtime_error("Reception failed: " + std::string(e.what()));
    }

    // 2. We deserialize the message in SystemState
    flr_api::v2::rtos::SystemState sys_state_msg;
    if (!sys_state_msg.ParseFromString(msg)) {
        throw std::runtime_error("Failed to parse SystemState message.");
    }

    return sys_state_msg;
}

Command::Command()// constructor
	:command_fd_(-1)
{
	//rien à faire ici; fd non initialisé, on ne s'en sert pas encore

}
	
bool Command::init_driver(Socket& socket)
{
	flr_api::v2::rtos::RTOSDriverInitializationCmd rtos_init_cmd;
    rtos_init_cmd.set_dummy_mode(false);
    rtos_init_cmd.set_robot_brand(robot_.brand);
    rtos_init_cmd.set_robot_model(robot_.model);
    rtos_init_cmd.set_robot_controller_model(robot_.controller_model);
    // rtos_init_cmd.set_local_address("192.168.10.42"); ça je sais pas si je laisse ça ou si "0.0.0.0". Mis en commentaire selon exemple n°2 de Fuzzy
    // rtos_init_cmd.set_robot_address("192.168.10.254"); Mis en commentaire selon exemple n°2 de Fuzzy
    std::cout << "init driver\n";
    const auto payload = asAny(rtos_init_cmd).SerializeAsString();
    
    // encapsulation of error messages, we catch the error mesage defined in socket.cpp functions and add precisions
    // on the error context. Then we throw again the enriched error that will be catched in the main.cpp
    
    try {
    	socket.send(payload);

    } catch (const std::exception& e){

    	throw std::runtime_error("failed during init driver: " + std::string(e.what()));
    }
    


    is_initialized_driver_ = true;
    return true;
}

bool Command::reset_driver(Socket& socket)
{
	flr_api::v2::rtos::RTOSDriverInitializationCmd rtos_init_cmd;
    std::cout << "Resetting driver\n";
    const auto payload = asAny(rtos_init_cmd).SerializeAsString();
    
    try {

    	socket.send(payload);

    } catch (const std::exception& e){

    	throw std::runtime_error("failed during reset driver: " + std::string(e.what()));
    };

    is_reset_driver_ = true;
    return true;
}

int Command::control_mode(Socket& socket, ControlMode mode)
{	
	const auto sys_state_msg = Command::receive_system_state(socket);

	// Depending of the mode, we send the desired mode
	switch(mode)
	{
		case ControlMode::jogging:
			if(sys_state_msg.rtos_state().robot_control_mode() != flr_api::v2::rtos::RTOSControlMode::RTOS_CONTROL_MODE_JOGGING)
			{
				flr_api::v2::rtos::RTOSControlModeCmd jog_mode_msg;
				jog_mode_msg.set_control_mode_request(flr_api::v2::rtos::RTOSControlMode::RTOS_CONTROL_MODE_JOGGING);
				const auto payload = asAny(jog_mode_msg).SerializeAsString();

				try {
    				socket.send(payload);
    				selected_mode_=mode;
    				return 1;

    			} catch (const std::exception& e){

    				throw std::runtime_error("failed to switch to jogging mode : " + std::string(e.what()));
    			}

			}
			return 0;

		default :
			throw std::invalid_argument("Unsupported control mode requested");


	} 

	return -1;

}

std::vector<double> Command::get_joint_position(Socket& socket)
{
	const auto sys_state_msg = Command::receive_system_state(socket);
	std::vector<double> joints_position;

	// we acquire the current positions
	const auto& current_joints_values = sys_state_msg.robot_state().joint_positions();
	// We need to define the size of joints_positions if we don't want an error to occur
	joints_position.resize(current_joints_values.value_size());

	for(int i=0; i < current_joints_values.value_size();++i)
        {
            joints_position[i] = current_joints_values.value(i);
        }

	return joints_position;
} 

std::vector<double> Command::get_tool_position(Socket& socket)
{
	const auto sys_state_msg = Command::receive_system_state(socket);

	const auto& pose = sys_state_msg.robot_state().tool_pose();

	double x = pose.translation().x();
	double y = pose.translation().y();
	double z = pose.translation().z();
	double qx = pose.rotation().qx();
	double qy = pose.rotation().qy();
	double qz = pose.rotation().qz();
	double qw = pose.rotation().qw();

	std::vector<double> tool_position = {x, y, z, qx, qy, qz, qw};

	return tool_position;

} 


void Command::compute_jacobian(std::vector<double> joint_position)
{

}

void Command::compute_error()
{

}


void Command::send_joint_position(int joint_index, double delta)
{

}

void Command::send_tool_position()
{

}

void Command::print_joint_position(Socket& socket) {
    const auto positions = get_joint_position(socket);
    std::cout << "Joint positions: ";
    for (const auto& val : positions)
        std::cout << val << " ";
    std::cout << std::endl;
}

void Command::print_tool_position(Socket& socket) {
    const auto pose = get_tool_position(socket);
    std::cout << "Tool pose (x y z qx qy qz qw): ";
    for (const auto& val : pose)
        std::cout << val << " ";
    std::cout << std::endl;
}

void Command::print_system_state(Socket& socket) {
    // Used for debbuging by printing all the system state
    const auto sys_state = receive_system_state(socket);
    std::cout << sys_state.DebugString();  // Affiche tout
}