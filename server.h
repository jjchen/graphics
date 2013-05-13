int server_init(int port);
char* server_receive(int socket_descriptor);
int server_reply();
int server();
void server_close(int socket_desc);
