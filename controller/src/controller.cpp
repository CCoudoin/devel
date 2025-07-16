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



Command::Command()//constructor
{

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
    


    is_initialized_driver = true;
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

    	throw st::runtime_error("failed during reset driver: " + std::string(e.what()));
    };

    is_reset_driver = true;
    return true;
}

int Command::control_mode(Socket& socket; ControlMode mode)
{	
	// 1. Reception of protobuf message
	std:string msg;
	
	try {
		socket.receive(msg,1024,2);

	} catch(const std::exception& e) {

		throw std::runtime_error("message reception failed during control mode selection : " + std::string(e.what()));
	}

	// 2. We deserialize the message in SystemState
	flr_api::v2::rtos::SystemState sys_state_msg;
	if(!sys_state_msg.ParseFromString(msg)){
		throw std::runtime_error("Failed to parse SystemState message from received data");
	}

	// 3. Depending of the mode, we send the desired mode
	switch(mode)
	{
		case jogging:
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

std::vector<double> get_joint_position(Socket& socket)
{
	flr_api::v2::rtos::SystemState sys_state_msg;
	socket.receive()
	std::cout << "Joint Pose before = " << sys_state_msg.robot_state().joint_positions().ShortDebugString() << std::endl;
}

void Command::compute_jacobian()
{

}

void Command::compute_error()
{

}


void Command::send_joint_position(int joint_index, double delta)
{

}