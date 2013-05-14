int server_init(int port);
char* server_receive(int socket_descriptor);
int server_write(char* buff, int total_count);
int server();
void server_close(int socket_desc);
