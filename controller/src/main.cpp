#include "socket.hpp"
#include "controller.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <csignal>
#include <chrono>
#include <thread>
#include <iostream>

bool keep_running = true;

void sigHandler(int /* sig */)
{
    keep_running = false;
}

int main()
{
	// creation of the socket and its configuration (struct)
	SocketConfig cfg;
	Socket sock(cfg);
	try {
		sock.connect();
	} catch (const std::exception& e){
		std::cerr << "Failed to connect: " << e.what() << std::endl;
	} 
	
	// Command of the robot
	//====================================================================================================
	// /!\ Be careful that the sent_tool_position() delta variable used to move the robot is in meters !
	//====================================================================================================

	Command cmd;
	try {
		cmd.init_driver(sock);
		cmd.control_mode(sock,ControlMode::jogging);

		std::cout << "========tool position (before)=========" << std::endl;
		cmd.print_tool_position(sock);

		

		//displacement of 5mm
		double dz=-0.005;
		cmd.send_tool_position(Direction::z,sock,dz);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "========tool position (after)=========" << std::endl;
		cmd.print_tool_position(sock);
		cmd.print_system_state(sock);

		cmd.reset_driver(sock);
	} catch (const std::exception& e){
		std::cerr << "command failed : " << e.what() << std::endl;
	}
	sock.close();


	//while(keep_running)
	//{
		// prise du temps pour calcul de la fréquence de réponse
		//cmd.get_joint_position(); //On regarde la position actuelle
		// cmd.compute_error(); On regarde l'erreur avec la consigne
		// cmd.compute_jacobian(); // On calcul la jacobienne pour corriger l'erreur
		// cmd.send_joint_position(); // On demande de nouvelles positions aux joints
	//}
		

}