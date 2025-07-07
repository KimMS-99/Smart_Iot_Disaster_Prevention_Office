#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <signal.h>
#include <mysql/mysql.h>

#define BUF_SIZE 100
#define NAME_SIZE 20
#define ARR_CNT 5

void* send_msg(void* arg);
void* recv_msg(void* arg);
void error_handling(char* msg);

char name[NAME_SIZE] = "[Default]";
char msg[BUF_SIZE];

int main(int argc, char* argv[])
{
	int sock;
	struct sockaddr_in serv_addr;
	pthread_t snd_thread, rcv_thread, mysql_thread;
	void* thread_return;

	if (argc != 4) {
		printf("Usage : %s <IP> <port> <name>\n", argv[0]);
		exit(1);
	}

	sprintf(name, "%s", argv[3]);

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock == -1)
		error_handling("socket() error");

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = inet_addr(argv[1]);
	serv_addr.sin_port = htons(atoi(argv[2]));

	if (connect(sock, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) == -1)
		error_handling("connect() error");

	sprintf(msg, "[%s:PASSWD]", name);
	write(sock, msg, strlen(msg));
	pthread_create(&rcv_thread, NULL, recv_msg, (void*)&sock);
	pthread_create(&snd_thread, NULL, send_msg, (void*)&sock);


	pthread_join(snd_thread, &thread_return);
	pthread_join(rcv_thread, &thread_return);

	close(sock);
	return 0;
}


void* send_msg(void* arg)
{
	int* sock = (int*)arg;
	int str_len;
	int ret;
	fd_set initset, newset;
	struct timeval tv;
	char name_msg[NAME_SIZE + BUF_SIZE + 2];

	FD_ZERO(&initset);
	FD_SET(STDIN_FILENO, &initset);

	fputs("Input a message! [ID]msg (Default ID:ALLMSG)\n", stdout);
	while (1) {
		memset(msg, 0, sizeof(msg));
		name_msg[0] = '\0';
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		newset = initset;
		ret = select(STDIN_FILENO + 1, &newset, NULL, NULL, &tv);
		if (FD_ISSET(STDIN_FILENO, &newset))
		{
			fgets(msg, BUF_SIZE, stdin);
			if (!strncmp(msg, "quit\n", 5)) {
				*sock = -1;
				return NULL;
			}
			else if (msg[0] != '[')
			{
				strcat(name_msg, "[ALLMSG]");
				strcat(name_msg, msg);
			}
			else
				strcpy(name_msg, msg);
			if (write(*sock, name_msg, strlen(name_msg)) <= 0)
			{
				*sock = -1;
				return NULL;
			}
		}
		if (ret == 0)
		{
			if (*sock == -1)
				return NULL;
		}
	}
}

void* recv_msg(void* arg)
{
	MYSQL* conn;
	MYSQL_ROW sqlrow;
	MYSQL_RES *result;
	int res;
	char sql_cmd[200] = { 0 };		
	char value[200];
	value[0] = '\0';
	char name[200];
	name[0] = '\0';
	int total_member;
	char* host = "localhost";
	char* user = "iot";
	char* pass = "pwiot";
	char* dbname = "minidb";

	int* sock = (int*)arg;
	int i;
	char* pToken;
	char* pArray[ARR_CNT] = { 0 };

	char name_msg[NAME_SIZE + BUF_SIZE + 1];
	char return_msg[NAME_SIZE + BUF_SIZE + 1];
	return_msg[0] = '\0';
	int str_len;

	int illu;
	float temp;
	float humi;
	conn = mysql_init(NULL);

	puts("MYSQL startup");
	if (!(mysql_real_connect(conn, host, user, pass, dbname, 0, NULL, 0)))
	{
		fprintf(stderr, "ERROR : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		exit(1);
	}
	else
		printf("Connection Successful!\n\n");

	while (1) {
		memset(name_msg, 0x0, sizeof(name_msg));
		str_len = read(*sock, name_msg, NAME_SIZE + BUF_SIZE);
		if (str_len <= 0)
		{
			*sock = -1;
			return NULL;
		}
		fputs(name_msg, stdout);
		//name_msg[str_len - 1] = 0;
		name_msg[strcspn(name_msg, "\n")] = '\0';
		//fputs(name_msg, stdout);
		

		pToken = strtok(name_msg, "[:@]");
		i = 0;
		while (pToken != NULL)
		{
			pArray[i] = pToken;
			//printf("%s\n", pArray[i]);
			if ( ++i >= ARR_CNT)
				break;
			pToken = strtok(NULL, "[:@]");

		}
/****************************************************SENSOR*********************************************/
		if(!strcmp(pArray[1],"SENSOR") && (i == 5)){
			illu = atoi(pArray[2]);
			temp = atof(pArray[3]);
			humi = atof(pArray[4]);
  			sprintf(sql_cmd, "insert into sensor(name, date, time,illu, temp, humi) values(\"%s\",now(),now(),%d,%f,%f)",pArray[0],illu, temp, humi);
			res = mysql_query(conn, sql_cmd);
			if (!res)
				printf("inserted %lu rows\n", (unsigned long)mysql_affected_rows(conn));
			else
				fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));

		}
/*********************************************GETDB****************************************************/
		else if(!strcmp(pArray[1],"GETDB") && (i == 3)){
			// 받은 id값에 해당하는 멤버의 출퇴근 상태와 이름을 DB에서 가져온다
			sprintf(sql_cmd, "select state, name from member where id='%s'", pArray[2]);
			res = mysql_query(conn, sql_cmd);
			if (!res) // 쿼리 성공 시  
			{	
				result = mysql_store_result(conn);
				if (mysql_num_rows(result) > 0) // select한 값이 존재 할 때(등록된 카드)
				{
					MYSQL_ROW row = mysql_fetch_row(result);
					strcpy(value, row[0]);
					strcpy(name, row[1]);
					sprintf(return_msg, "[%s]%s@%s@%s@%s@%s\n",pArray[0], pArray[1], pArray[2], "YES", value, name); 
					write(*sock, return_msg, strlen(return_msg));
					if (!strcmp(value, "0"))
					{
						sprintf(sql_cmd, "update member set state=1 where id='%s'", pArray[2]);
						mysql_query(conn, sql_cmd);
					}
					else if (!strcmp(value, "1"))
					{					
						sprintf(sql_cmd, "update member set state=0 where id='%s'", pArray[2]);
						mysql_query(conn, sql_cmd);
					}
					mysql_free_result(result);
				}
				else // select한 값이 존재 하지 않을 때(등록되지 않은 카드)
				{
					sprintf(return_msg, "[%s]%s@%s@%s\n",pArray[0], pArray[1], pArray[2], "NO");
					write(*sock, return_msg, strlen(return_msg)); 
				}
			}
			else // 쿼리 실패 시
			{
				fprintf(stderr, "ERROR: %s[%d]\n", mysql_error(conn), mysql_errno(conn));
			}
		}
/************************************************SETDB*****************************************************/
		else if (!strcmp(pArray[1], "SETDB") && (i == 4)) {
			if (!strcmp(pArray[2], "LAMP")||!strcmp(pArray[2], "WINDOW")||!strcmp(pArray[2], "AC")||!strcmp(pArray[2], "GAS")||!strcmp(pArray[2], "ELEC")||!strcmp(pArray[2], "TOTAL")) 
			{
				sprintf(sql_cmd, "update state set %s='%s' where name='KMW_SQL'", pArray[2],pArray[3]);
				mysql_query(conn, sql_cmd);
				printf("update complete!!\t%s -> %s\n",pArray[2], pArray[3]);
			}
			else if(!strcmp(pArray[2], "INIT") && !strcmp(pArray[3], "STATE"))
			{

				sprintf(sql_cmd, "update member set state = 0");
                mysql_query(conn, sql_cmd);
			}
		}
		else if(!strcmp(pArray[1], "SETDB") && (i == 5))
		{
			if(!strcmp(pArray[2], "DHT"))
			{
				float h, t;
				h = atof(pArray[3]);
				t = atof(pArray[4]);
				sprintf(sql_cmd, "update state set HUMI = %f, TEMP = %f where name = 'KMW_SQL'", h, t);
				mysql_query(conn, sql_cmd);
			}
		}
		
		else if(!strcmp(pArray[1], "CHEC") && (i == 2))
		{
			sprintf(sql_cmd, "select TEMP, HUMI, AC, TOTAL from state");
			res = mysql_query(conn, sql_cmd);
			if(!res)
			{
				printf("CHECK\n");
				result = mysql_store_result(conn);
				mysql_num_rows(result);
				MYSQL_ROW row = mysql_fetch_row(result);
				sprintf(return_msg, "[%s]AC-%s, TOTAL-%s, TEMP-%s, HUMI-%s \n",pArray[0], row[2], row[3], row[0], row[1]);
				write(*sock, return_msg, strlen(return_msg)); 
			}
		}
	}
	mysql_close(conn);
}


void error_handling(char* msg)
{
	fputs(msg, stderr);
	fputc('\n', stderr);
	exit(1);
}
