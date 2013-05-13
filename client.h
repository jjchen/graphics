int client_init(char* server_addr, int port);
int client_write(char* data, int sd);
char* client_read(int sd);
void client_close(int sd);
