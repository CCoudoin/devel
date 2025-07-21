#include "socket.hpp"
#include "controller.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <csignal>
#include <chrono>
#include <ctime>
#include <thread>
#include <iostream>
#include <fstream>

#include <csignal> // For the signal function to exit the while loop with CTRL+C

// pour fonction sinusoidale
#define _USE_MATH_DEFINES
#include <cmath> //to use sin function

// atomic_bool is used because keep_running will be used in the while loop and in the sigHandler function.
// It could cause an issue of access to the data depending of which is prioritized by the compiler.
// with atomic bool, the compiler doesn't do dangerous optimisation and it will read each time the value allocated in memory
std::atomic_bool keep_running = true; 

void sigHandler(int /* sig */)
{
    // This function is to be able to exit the while loop by pressing CTRL+C
    keep_running = false;
}

double sinusoid_movement (double amplitude_max, float t_elapsed)
{
	//this function will return a delta that will be given to the send_tool_position
	// it will be used to caracterize the reponse frequency of the robot

	//--------parameters for sinusoidal-----------------
		//Attention à l'unité amplitude : radian si utilisé pour mvt articulaire mètre si cartésien
		// 0.05 en radian -> (delta*180)/Pi pour avoir en degré soit ici environ 3°
		// 0.005 en m -> 5mm

        double frequence_sin = 0.2; //frequence en Hz du signal pour le mouvement
        double omega = 2 * M_PI * frequence_sin;
        double sin_delta = amplitude_max * sin(omega*t_elapsed); //delta en radian que l'on va appliquer au joint de l'index précédent. (delta*180)/Pi pour avoir en degré
        
        return sin_delta;
}

int main()
{
	// log file
	std::ofstream logfile("cmd_trajectory.csv");
	logfile << "time,dz"<<std::endl;

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
	} catch (const std::exception& e){
		std::cerr << "main : command preparatin failed : " << e.what() << std::endl;
	}


	clock_t before = clock();
	while(keep_running)
	{
		try {


			std::cout << "========tool position (before)=========" << std::endl;
			cmd.print_tool_position(sock);

			// displacement of 5mm
			// double dz=-0.005;
			//===parameters for the sinusoidal mouvement===
			double amplitude_max = 0.005;
			clock_t t_elapsed = clock()-before;
			// ============================================

			double dz = sinusoid_movement(amplitude_max,t_elapsed);
			logfile << t_elapsed << "," << dz << std::endl;
			cmd.send_tool_position(Direction::z,sock,dz);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

			std::cout << "========tool position (after)=========" << std::endl;
			cmd.print_tool_position(sock);
			cmd.print_system_state(sock);
			
			// programm execution duration
			//clock_t duration = clock()-before;
			std::cout << "Duration : " << (float)t_elapsed / CLOCKS_PER_SEC << " seconds";

			

		} catch (const std::exception& e){
				std::cerr << "command failed : " << e.what() << std::endl;
		}

	}
	// juste pour tester si fonctionne en dehors de boucle while
	cmd.print_tool_position(sock);
	cmd.print_system_state(sock);
	//-------
	cmd.reset_driver(sock);
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