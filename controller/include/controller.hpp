//controller header
// Command manipulates only the protobuf messages, these messages are then serialized as string in main .cpp and send thanks to sockets properties.
#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <string>
#include <netinet/in.h> // sockaddr_in
#include <sys/socket.h> // socket, bind, recvfrom
#include <arpa/inet.h>  // inet_addr
#include <unistd.h>     // close

 class Robot {
 	public :
 	std::string brand = "Staubli";
 	std::string controller = "TX2-60L";
 	std::string model = "CS9";
 	std::string address = "192.168.10.254";
 	// values to define the robot's cell limitation
 	// float x_axis_user_min
 	// float x_axis_user_max
 	// float y_axis_user_min
 	// float y_axis_user_max
 	// float z_axis_user_min
 	// float z_axis_user_max
 	float joint_min_position = 0;// valeur par défaut présente dans le code de Fuzzy Logic
 	float joint_max_position = 1000;// valeur par défaut présente dans le code de Fuzzy Logic
 	float max_velocity = 1000;// valeur par défaut présente dans le code de Fuzzy Logic
 	float max_acceleration = 5000;// valeur par défaut présente dans le code de Fuzzy Logic
 	float max_jerk = 10000;// valeur par défaut présente dans le code de Fuzzy Logic
};

class Command {
	public :
	Command();//constructor : Automatically called when an object is created

	bool init_driver();
	bool reset_driver();
	std::vector<double> get_joint_position();
	void compute_jacobian();
	void compute_error();
	void send_joint_position(int joint_index, double delta);

	private :
	Robot robot_;

};

#endif