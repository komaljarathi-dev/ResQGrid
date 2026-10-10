/*
 * EHACK - QNX Raspberry Pi 4 native application
 *
 * Ambulance ESP8266 -> QNX TCP server :5000
 * QNX native IPC -> decision thread
 * Decision thread -> Intersection ESP8266 TCP :5001
 */

#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/siginfo.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define AMBULANCE_PORT       5000
#define CONTROLLER_IP        "192.168.0.131"
#define CONTROLLER_PORT      5001
#define MSG_TEXT_SIZE        128
#define TCP_LINE_SIZE        256
#define PRIORITY_THRESHOLD   5
#define TIMER_PULSE_CODE     (_PULSE_CODE_MINAVAIL + 1)
#define TIMER_INTERVAL_SEC   2
#define PRIORITY_HOLD_SEC    30

#define I1_LAT 17.3850
#define I1_LON 78.4867
#define I2_LAT 17.3900
#define I2_LON 78.4900
#define I3_LAT 17.3800
#define I3_LON 78.4800
#define I4_LAT 17.3950
#define I4_LON 78.4800

typedef struct {
    int type;
    char text[MSG_TEXT_SIZE];
    int level;
    double lat;
    double lon;
} Request;

typedef struct {
    int status;
    char text[MSG_TEXT_SIZE];
} Reply;

static volatile sig_atomic_t running = 1;
static int decision_channel = -1;
static int timer_channel = -1;
static int connection_id = -1;
static timer_t timer_id;
static int timer_created = 0;
static int timer_connection_id = -1;

enum {
    REQUEST_AMBULANCE = 1,
    REQUEST_HOSPITAL_REACHED = 2
};

static const double intersection_lat[4] = {
    I1_LAT, I2_LAT, I3_LAT, I4_LAT
};

static const double intersection_lon[4] = {
    I1_LON, I2_LON, I3_LON, I4_LON
};

/* Road graph:
 * I1 <-> I2, I1 <-> I3, I2 <-> I4, I3 <-> I4
 */
static const int road_graph[4][4] = {
    {0, 1, 1, 0},
    {1, 0, 0, 1},
    {1, 0, 0, 1},
    {0, 1, 1, 0}
};

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

static double degrees_to_radians(double degrees)
{
    return degrees * 0.017453292519943295;
}

static double distance_km(double lat1, double lon1,
                          double lat2, double lon2)
{
    const double earth_radius_km = 6371.0;
    double dlat = degrees_to_radians(lat2 - lat1);
    double dlon = degrees_to_radians(lon2 - lon1);

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(degrees_to_radians(lat1)) *
               cos(degrees_to_radians(lat2)) *
               sin(dlon / 2.0) * sin(dlon / 2.0);

    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;

    return earth_radius_km * 2.0 *
           atan2(sqrt(a), sqrt(1.0 - a));
}

static int nearest_intersection(double lat, double lon)
{
    int best = 0;
    double best_distance = distance_km(
        lat, lon, intersection_lat[0], intersection_lon[0]);

    for (int i = 1; i < 4; ++i) {
        double d = distance_km(
            lat, lon, intersection_lat[i], intersection_lon[i]);

        if (d < best_distance) {
            best_distance = d;
            best = i;
        }
    }

    return best;
}

/* Return the first hop on the shortest path from start to I4.
 * Returns -1 if no route exists or start is already I4.
 */
static int next_hop_to_hospital(int start)
{
    int queue[4];
    int head = 0, tail = 0;
    int visited[4] = {0, 0, 0, 0};
    int parent[4] = {-1, -1, -1, -1};
    const int target = 3;

    if (start < 0 || start >= 4 || start == target)
        return -1;

    queue[tail++] = start;
    visited[start] = 1;

    while (head < tail) {
        int current = queue[head++];

        if (current == target)
            break;

        for (int next = 0; next < 4; ++next) {
            if (road_graph[current][next] && !visited[next]) {
                visited[next] = 1;
                parent[next] = current;
                queue[tail++] = next;
            }
        }
    }

    if (!visited[target])
        return -1;

    {
        int node = target;

        while (parent[node] != -1 && parent[node] != start)
            node = parent[node];

        return node;
    }
}

static int send_priority_command(int intersection_index)
{
    int sock = -1;
    struct sockaddr_in address;
    char command[64];
    char response[128];
    size_t used = 0;
    int result = -1;
    struct timeval timeout;

    if (intersection_index < 0 || intersection_index > 3) {
        fprintf(stderr, "[CONTROLLER] Invalid intersection index\n");
        return -1;
    }

    if (intersection_index == 3) {
        printf("[ROUTE] Already at hospital intersection I4; no controller command sent.\n");
        return 0;
    }

    (void)snprintf(command, sizeof(command),
                   "PRIORITY,I%d,GREEN\n", intersection_index + 1);

    sock = socket(AF_INET, SOCK_STREAM, 0);

    if (sock < 0) {
        perror("[CONTROLLER] socket");
        return -1;
    }

    timeout.tv_sec = 4;
    timeout.tv_usec = 0;

    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout));
    (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO,
                     &timeout, sizeof(timeout));

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(CONTROLLER_PORT);

    if (inet_pton(AF_INET, CONTROLLER_IP, &address.sin_addr) != 1) {
        fprintf(stderr, "[CONTROLLER] Invalid controller IP address\n");
        close(sock);
        return -1;
    }

    printf("[CONTROLLER] Connecting to %s:%d...\n",
           CONTROLLER_IP, CONTROLLER_PORT);
    fflush(stdout);

    if (connect(sock, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("[CONTROLLER] connect");
        close(sock);
        return -1;
    }

    printf("[CONTROLLER] Connected. Sending: %s", command);
    fflush(stdout);

    {
        size_t total = strlen(command);
        size_t sent = 0;

        while (sent < total) {
            ssize_t n = send(sock, command + sent, total - sent, 0);

            if (n < 0) {
                if (errno == EINTR) continue;
                perror("[CONTROLLER] send");
                close(sock);
                return -1;
            }

            if (n == 0) {
                fprintf(stderr, "[CONTROLLER] send returned 0\n");
                close(sock);
                return -1;
            }

            sent += (size_t)n;
        }
    }

    while (used < sizeof(response) - 1) {
        char ch;
        ssize_t n = recv(sock, &ch, 1, 0);

        if (n == 0)
            break;

        if (n < 0) {
            if (errno == EINTR) continue;

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fprintf(stderr, "[CONTROLLER] Timed out waiting for ACK\n");
            } else {
                perror("[CONTROLLER] recv");
            }
            break;
        }

        if (ch == '\r') continue;
        if (ch == '\n') break;

        response[used++] = ch;
    }

    response[used] = '\0';

    if (used > 0) {
        printf("[CONTROLLER] Response: %s\n", response);

        if (strncmp(response, "ACK,", 4) == 0)
            result = 0;
    } else {
        fprintf(stderr, "[CONTROLLER] No complete ACK received\n");
    }

    close(sock);
    return result;
}
static int submit_request(const Request *request, Reply *reply)
{
    int rc;

    if (connection_id < 0) {
        snprintf(reply->text, sizeof(reply->text), "ERROR,QNX_IPC_NOT_READY");
        reply->status = -1;
        return -1;
    }

    memset(reply, 0, sizeof(*reply));

    rc = MsgSend(connection_id, request, sizeof(*request),
                 reply, sizeof(*reply));

    if (rc == -1) {
        perror("[QNX IPC] MsgSend");
        snprintf(reply->text, sizeof(reply->text), "ERROR,QNX_IPC_SEND");
        reply->status = -1;
        return -1;
    }

    return 0;
}

static void *decision_thread(void *arg)
{
    (void)arg;

    printf("[DECISION] Thread started; waiting for native IPC messages.\n");
    fflush(stdout);

    while (running) {
        Request request;
        struct _msg_info info;

        int rc = MsgReceive(decision_channel, &request,
                            sizeof(request), &info);

        if (rc == -1) {
            if (errno == EINTR && !running) break;
            if (errno == EINTR) continue;

            perror("[DECISION] MsgReceive");
            continue;
        }

        if (rc == 0) {
            struct _pulse *pulse = (struct _pulse *)&request;

            if (pulse->code == TIMER_PULSE_CODE) {
                printf("[TIMER] 2-second timer pulse received.\n");
                fflush(stdout);
            }

            continue;
        }

        {
            Reply reply;
            memset(&reply, 0, sizeof(reply));

            if (request.type == REQUEST_HOSPITAL_REACHED ||
                strcmp(request.text, "HOSPITAL_REACHED") == 0) {

                snprintf(reply.text, sizeof(reply.text),
                         "ACK,HOSPITAL_REACHED");
                reply.status = 0;

                printf("[DECISION] Ambulance reports hospital reached.\n");

            } else if (request.type == REQUEST_AMBULANCE ||
                       strcmp(request.text, "AMBULANCE") == 0) {

                printf("[DECISION] Level=%d GPS=%.6f,%.6f\n",
                       request.level, request.lat, request.lon);

                if (request.level < 1 || request.level > 9 ||
                    !isfinite(request.lat) || !isfinite(request.lon) ||
                    request.lat < -90.0 || request.lat > 90.0 ||
                    request.lon < -180.0 || request.lon > 180.0) {

                    snprintf(reply.text, sizeof(reply.text),
                             "ERROR,INVALID_AMBULANCE_DATA");
                    reply.status = -1;

                } else if (request.level < PRIORITY_THRESHOLD) {

                    snprintf(reply.text, sizeof(reply.text),
                             "ACK,NO_PRIORITY,LEVEL=%d", request.level);
                    reply.status = 0;

                    printf("[DECISION] Level below threshold %d; no signal priority.\n",
                           PRIORITY_THRESHOLD);

                } else {
                    int nearest = nearest_intersection(
                        request.lat, request.lon);

                    int next = next_hop_to_hospital(nearest);

                    printf("[ROUTE] Nearest=I%d, ", nearest + 1);

                    if (next >= 0)
                        printf("Next=I%d toward hospital I4\n", next + 1);
                    else
                        printf("No next hop available\n");

                    if (next < 0) {
                        snprintf(reply.text, sizeof(reply.text),
                                 "ERROR,NO_ROUTE_TO_HOSPITAL");
                        reply.status = -1;
                    } else {
                        int controller_rc = send_priority_command(next);

                        if (controller_rc == 0) {
                            snprintf(reply.text, sizeof(reply.text),
                                     "ACK,PRIORITY,I%d,GREEN", next + 1);
                            reply.status = 0;
                        } else {
                            snprintf(reply.text, sizeof(reply.text),
                                     "ERROR,CONTROLLER_NOT_ACKNOWLEDGED,I%d",
                                     next + 1);
                            reply.status = -1;
                        }
                    }
                }
            } else {
                snprintf(reply.text, sizeof(reply.text),
                         "ERROR,UNKNOWN_REQUEST");
                reply.status = -1;
            }

            if (MsgReply(rc, 0, &reply, sizeof(reply)) == -1) {
                perror("[DECISION] MsgReply");
            }

            fflush(stdout);
        }
    }

    return NULL;
}

static int receive_line(int sock, char *buffer, size_t capacity)
{
    size_t used = 0;

    if (capacity < 2)
        return -1;

    while (running && used < capacity - 1) {
        char ch;
        ssize_t n = recv(sock, &ch, 1, 0);

        if (n == 0) {
            printf("[TCP] Client closed connection before newline; bytes=%lu\n",
                   (unsigned long)used);
            break;
        }

        if (n < 0) {
            if (errno == EINTR) {
                if (!running) return -1;
                continue;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fprintf(stderr,
                        "[FAULT] Timed out waiting for ambulance message.\n");
                return -2;
            }

            perror("[FAULT] TCP recv");
            return -1;
        }

        if (ch == '\r')
            continue;

        if (ch == '\n')
            break;

        buffer[used++] = ch;
    }

    buffer[used] = '\0';

    if (used == 0)
        return -1;

    /* Reject a full buffer instead of processing a truncated command. */
    if (used == capacity - 1) {
        fprintf(stderr,
                "[FAULT] Ambulance message exceeded maximum length.\n");
        return -3;
    }

    return 0;
}

static void process_ambulance_message(int client_sock, const char *line)
{
    Request request;
    Reply reply;
    int parsed;

    printf("[TCP] Received (%lu bytes): \"%s\"\n",
           (unsigned long)strlen(line), line);
    fflush(stdout);

    memset(&request, 0, sizeof(request));
    memset(&reply, 0, sizeof(reply));

    if (strcmp(line, "HOSPITAL_REACHED") == 0) {
        request.type = REQUEST_HOSPITAL_REACHED;
        snprintf(request.text, sizeof(request.text), "HOSPITAL_REACHED");
    } else {
        int consumed = 0;

        parsed = sscanf(line,
                        "AMBULANCE,LEVEL=%d,LAT=%lf,LON=%lf%n",
                        &request.level, &request.lat, &request.lon,
                        &consumed);

        if (parsed == 3 && line[consumed] != '\0')
            parsed = 0;

        if (parsed != 3) {
            fprintf(stderr,
                    "[TCP] Invalid message format: \"%s\"\n", line);

            snprintf(reply.text, sizeof(reply.text),
                     "ERROR,INVALID_MESSAGE_FORMAT");
            reply.status = -1;

            (void)send(client_sock, reply.text, strlen(reply.text), 0);
            (void)send(client_sock, "\n", 1, 0);
            return;
        }

        request.type = REQUEST_AMBULANCE;
        snprintf(request.text, sizeof(request.text), "AMBULANCE");
    }

    printf("[TCP] Passing request to QNX native IPC...\n");
    fflush(stdout);

    if (submit_request(&request, &reply) != 0) {
        fprintf(stderr, "[TCP] QNX IPC request failed\n");
        snprintf(reply.text, sizeof(reply.text), "ERROR,QNX_IPC_FAILED");
    }

    printf("[TCP] Sending reply to ambulance: %s\n", reply.text);
    fflush(stdout);

    {
        char outgoing[MSG_TEXT_SIZE + 2];
        size_t sent = 0;

        (void)snprintf(outgoing, sizeof(outgoing), "%s\n", reply.text);
        size_t total = strlen(outgoing);

        while (sent < total) {
            ssize_t n = send(client_sock, outgoing + sent, total - sent, 0);

            if (n < 0) {
                if (errno == EINTR) continue;
                perror("[TCP] send reply");
                break;
            }

            if (n == 0) break;

            sent += (size_t)n;
        }
    }
}
static int start_tcp_server(void)
{
    int server_sock;
    int reuse = 1;
    struct sockaddr_in address;

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        perror("[TCP] socket");
        return -1;
    }

    (void)setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR,
                     &reuse, sizeof(reuse));

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(AMBULANCE_PORT);

    if (bind(server_sock, (struct sockaddr *)&address,
             sizeof(address)) < 0) {
        perror("[TCP] bind");
        close(server_sock);
        return -1;
    }

    if (listen(server_sock, 8) < 0) {
        perror("[TCP] listen");
        close(server_sock);
        return -1;
    }

    printf("[TCP] Ambulance server listening on port %d\n",
           AMBULANCE_PORT);
    fflush(stdout);

    while (running) {
        struct sockaddr_in peer;
        socklen_t peer_len = sizeof(peer);
        int client_sock = accept(server_sock,
                                 (struct sockaddr *)&peer, &peer_len);

        if (client_sock < 0) {
            if (errno == EINTR && !running) break;
            if (errno == EINTR) continue;
            perror("[TCP] accept");
            continue;
        }

        printf("[TCP] Client connected from %s:%u\n",
               inet_ntoa(peer.sin_addr),
               (unsigned)ntohs(peer.sin_port));
        fflush(stdout);

        {
            struct timeval timeout;
            char line[TCP_LINE_SIZE];

            timeout.tv_sec = 10;
            timeout.tv_usec = 0;

            (void)setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO,
                             &timeout, sizeof(timeout));

            memset(line, 0, sizeof(line));

            {
                int line_rc = receive_line(
                    client_sock, line, sizeof(line));

                if (line_rc == 0) {
                    process_ambulance_message(client_sock, line);
                } else if (line_rc == -2) {
                    fprintf(stderr,
                            "[FAULT] No ambulance message from %s before timeout.\n",
                            inet_ntoa(peer.sin_addr));
                } else if (line_rc == -3) {
                    const char *err = "ERROR,MESSAGE_TOO_LONG\n";
                    (void)send(client_sock, err, strlen(err), 0);
                } else {
                    fprintf(stderr,
                            "[FAULT] No complete valid line received from %s; closing client.\n",
                            inet_ntoa(peer.sin_addr));
                }
            }
        }

        shutdown(client_sock, SHUT_RDWR);
        close(client_sock);

        printf("[TCP] Client connection closed.\n");
        fflush(stdout);
    }

    close(server_sock);
    return 0;
}

static int start_timer(void)
{
    struct sigevent event;
    struct itimerspec timer_spec;

    timer_connection_id = ConnectAttach(
        ND_LOCAL_NODE, 0, decision_channel, _NTO_SIDE_CHANNEL, 0);

    if (timer_connection_id == -1) {
        perror("[TIMER] ConnectAttach");
        return -1;
    }

    SIGEV_PULSE_INIT(&event, timer_connection_id,
                     SIGEV_PULSE_PRIO_INHERIT, TIMER_PULSE_CODE, 0);

    if (timer_create(CLOCK_MONOTONIC, &event, &timer_id) == -1) {
        perror("[TIMER] timer_create");
        ConnectDetach(timer_connection_id);
        timer_connection_id = -1;
        return -1;
    }

    timer_created = 1;
    memset(&timer_spec, 0, sizeof(timer_spec));

    timer_spec.it_value.tv_sec = TIMER_INTERVAL_SEC;
    timer_spec.it_interval.tv_sec = TIMER_INTERVAL_SEC;

    if (timer_settime(timer_id, 0, &timer_spec, NULL) == -1) {
        perror("[TIMER] timer_settime");
        timer_delete(timer_id);
        timer_created = 0;
        ConnectDetach(timer_connection_id);
        timer_connection_id = -1;
        return -1;
    }

    printf("[TIMER] Periodic timer configured: every %d seconds.\n",
           TIMER_INTERVAL_SEC);

    return 0;
}

int main(void)
{
    pthread_t decision_tid;
    struct sched_param decision_param;
    int decision_thread_started = 0;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("=============================================\n");
    printf(" EHACK QNX AMBULANCE INTERSECTION CONTROLLER\n");
    printf("=============================================\n");

    printf("[CONFIG] Ambulance TCP port: %d\n", AMBULANCE_PORT);
    printf("[CONFIG] Intersection controller: %s:%d\n",
           CONTROLLER_IP, CONTROLLER_PORT);
    printf("[CONFIG] Priority threshold: %d\n", PRIORITY_THRESHOLD);

    decision_channel = ChannelCreate(0);

    if (decision_channel == -1) {
        perror("[QNX IPC] ChannelCreate");
        return EXIT_FAILURE;
    }

    connection_id = ConnectAttach(
        ND_LOCAL_NODE, 0, decision_channel, _NTO_SIDE_CHANNEL, 0);

    if (connection_id == -1) {
        perror("[QNX IPC] ConnectAttach");
        ChannelDestroy(decision_channel);
        return EXIT_FAILURE;
    }

    if (pthread_create(&decision_tid, NULL,
                       decision_thread, NULL) != 0) {
        perror("[DECISION] pthread_create");
        ConnectDetach(connection_id);
        ChannelDestroy(decision_channel);
        return EXIT_FAILURE;
    }

    decision_thread_started = 1;

    memset(&decision_param, 0, sizeof(decision_param));
    decision_param.sched_priority = 20;

    if (pthread_setschedparam(decision_tid, SCHED_RR,
                              &decision_param) != 0) {
        fprintf(stderr,
                "[WARNING] Could not set decision thread SCHED_RR priority; continuing.\n");
    }

    (void)start_timer();

    printf("[READY] QNX application ready. Waiting for ambulance ESP8266...\n");
    printf("[READY] Send: AMBULANCE,LEVEL=8,LAT=17.385000,LON=78.486700\n");
    fflush(stdout);

    /*
     * Recover from transient TCP server startup failures.
     */
    while (running) {
        int server_rc = start_tcp_server();

        if (!running)
            break;

        if (server_rc == 0) {
            break;
        }

        fprintf(stderr,
                "[RECOVERY] TCP server failed; retrying in 3 seconds.\n");
        sleep(3);
    }

    running = 0;

    if (timer_created) {
        timer_delete(timer_id);
        timer_created = 0;
    }

    if (timer_connection_id >= 0) {
        ConnectDetach(timer_connection_id);
        timer_connection_id = -1;
    }

    if (connection_id >= 0)
        ConnectDetach(connection_id);

    if (decision_thread_started)
        pthread_join(decision_tid, NULL);

    if (decision_channel >= 0)
        ChannelDestroy(decision_channel);

    printf("[EXIT] EHACK QNX application stopped.\n");

    return EXIT_SUCCESS;
}
