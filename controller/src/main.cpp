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

#include <vector>
#include <string>

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

        const double frequency_hz = 0.2; //frequence en Hz du signal pour le mouvement
        const double omega = 2 * M_PI * frequency_hz;
        const double t_sec = t_elapsed / 1000.0;
        
        return amplitude_max * std::sin(omega*t_sec);
}

double up_down_movements (double amplitude_max, int iterations)
{
	// This function will return a delta that will be given to the send_tool_position
	// it will be used to do some tests on the robot's response	
		if (iterations%2 ==0)
		{
			return amplitude_max;
		} else
		{
			return - amplitude_max;
		}
	
}

// This struct is used to log data into a file.
struct LogEntry {
	double timestamp;
	double delta;
};

// This struct is used to log errors
struct ErrorLogEntry{
	double timestamp;
	std::string message;
};

double get_current_time()
{
    struct timeval current;
    gettimeofday(&current, nullptr);
    return current.tv_sec + current.tv_usec * 1e-6;
}

int main()
{
	//start time
	double start_time = get_current_time();

	// struct used to log errors
	std::vector<ErrorLogEntry> error_log;

	// struct used to log data
	std::vector<LogEntry> data_log;

	// To interupt with CTRL+C
	signal(SIGINT, sigHandler);

	// creation of the socket and its configuration (struct)
	SocketConfig cfg;
	Socket sock(cfg);
	try {
		sock.connect();
	} catch (const std::exception& e){
		// std::cerr << "Failed to connect: " << e.what() << std::endl;
		error_log.push_back({get_current_time(), "[ERROR] Failed to connect: " + std::string(e.what())});
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
		//std::cerr << "main : command preparation failed : " << e.what() << std::endl;
		error_log.push_back({get_current_time(),"[ERROR] Failed command preparation: " + std::string(e.what())});
	}
	while(keep_running)
	{
		auto loop_start = std::chrono::steady_clock::now();
		/*try {

			std::cout << "========tool position (before)=========" << std::endl;
			cmd.print_tool_position(sock);
		} catch (const std::exception& e){
				std::cerr << "printing of tool position before command failed : " << e.what() << std::endl;
		}*/

		//===parameters for the sinusoidal mouvement===
		double amplitude_max = 0.005; // unit is m -> displacement of 5mm
		auto command_time = get_current_time();
		auto t_elapsed = command_time - start_time;
			// ============================================

		try {

			double delta = sinusoid_movement(amplitude_max,t_elapsed);

			// logfile << t_elapsed << "," << dz << std::endl;
			data_log.push_back({get_current_time(),delta});

			cmd.send_tool_position(Direction::z,sock,delta);

		} catch (const std::exception& e){
				// std::cerr << "sending of tool position failed : " << e.what() << std::endl;
				error_log.push_back({get_current_time(),"[ERROR] Tool position sending failed : " + std::string(e.what())});
		}

		/* A voir comment je gère le sleep dans ce cas.. A creuser
		try {
			for (int i=0; i<10; ++i)
			{
				double dz = up_down_movements(0.005,i);
				cmd.send_tool_position(Direction::z,sock,dz);

			}
		} catch (const std::exception& e){
			std::cerr << "sending of tool position failed : " << e.what() << std::endl;
		}*/
		
		/*try {
			std::cout << "========tool position (after)=========" << std::endl;
			cmd.print_tool_position(sock);
		} catch (const std::exception& e){
				std::cerr << "printing of tool position after command failed : " << e.what() << std::endl;
		}*/


		// programm execution duration
		auto loop_end = std::chrono::steady_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
		if (duration.count() < 4)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(4 - duration.count()));// We want 250Hz which is 4ms
		} else
		{
			// std::cout << "WARNING: loop higher than expected duration!" << std::endl;
			error_log.push_back({get_current_time(),"[WARNING] loop duration higher than expected duration!"});
		}

	}


	try {
		cmd.reset_driver(sock);
		sock.close();
	} catch (const std::exception& e){
			// std::cerr << "ending of the programm failed : " << e.what() << std::endl;
			error_log.push_back({get_current_time(),e.what()});
	}

	// log file data
	std::ofstream logfile("cmd_trajectory.csv");
	for (const auto& e : error_log)
	{
		logfile << "[" << e.timestamp << "s] " << e.message << "\n";
	}


	// log file errors
	std::ofstream err_file("log_errors.txt");
	for (const auto& e : error_log)
	{
		err_file << "[" << e.timestamp << "s] " << e.message << "\n";
	}
	

}