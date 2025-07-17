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
	Command cmd;
	try {
		cmd.init_driver(sock);
		std::cout << "========joint position=========\n" << std::endl;
		cmd.print_joint_position(sock);
		std::cout << "========tool position=========\n" << std::endl;
		cmd.print_tool_position(sock);
		std::cout << "========system state=========\n" << std::endl;
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