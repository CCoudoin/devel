// socket header
 
#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <string>
#include <netinet/in.h> // sockaddr_in
#include <sys/socket.h> // socket, bind, recvfrom
#include <arpa/inet.h>  // inet_addr
#include <unistd.h>     // close


 struct SocketConfig {
 	int domain = AF_INET;
 	int type = SOCK_DGRAM;
 	int protocol = IPPROTO_UDP;
 	const std::string dest_adress = "192.168.10.254";
 	int dest_port = "51218";
 	const std::string local_adress = "192.168.10.42";
}

class Socket
{
	public:

		Socket();//constructor : Automatically called when an object is created

		Socket(int domain, int type, int protocol);//init socket

		void bind(string ip, string port);
		void connect(const SocketConfig& cfg);
		ssize_t send(const std::string& msg); //ssize_t permet de renvoyer la taille du message ou -1 pour savoir si il y a une erreur
		ssize_t receive(const std::string& msg, size_t max_length = 1024,int timeout_sec=2); //ssize_t permet de renvoyer la taille du message ou -1 pour savoir si il y a une erreur
		void close();

	private :

	int socket_fd_; //File Descriptor du socket (un entier retrouné par ::socket dans les autres fonctions)
	sockaddr_in remote_addr_; //Représentation système réutilisé dans les autres fonctions. Evite d'avoir à repasser par SocketConfig.
	bool is_connected_ = false;
}
#endif