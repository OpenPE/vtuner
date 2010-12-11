#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>
#include <linux/dvb/dmx.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>


#define VTUNER_GET_MESSAGE  1
#define VTUNER_SET_RESPONSE 2
#define VTUNER_SET_NAME     3
#define VTUNER_SET_TYPE     4
#define VTUNER_SET_HAS_OUTPUTS 5
#define VTUNER_SET_FE_INFO 6

#define MSG_SET_FRONTEND         1
#define MSG_GET_FRONTEND         2
#define MSG_READ_STATUS          3
#define MSG_READ_BER             4
#define MSG_READ_SIGNAL_STRENGTH 5
#define MSG_READ_SNR             6
#define MSG_READ_UCBLOCKS        7
#define MSG_SET_TONE             8
#define MSG_SET_VOLTAGE          9
#define MSG_ENABLE_HIGH_VOLTAGE  10
#define MSG_SEND_DISEQC_MSG      11
#define MSG_SEND_DISEQC_BURST    13
#define MSG_PIDLIST              14

struct vtuner_message
{
	int type;
	union
	{
		struct dvb_frontend_parameters dvb_frontend_parameters;
		fe_status_t status;
		__u32 ber;
		__u16 ss, snr;
		__u32 ucb;
		fe_sec_tone_mode_t tone;
		fe_sec_voltage_t voltage;
		fe_sec_mini_cmd_t burst;
		__u16 pidlist[30];
		unsigned char pad[60];
	} body;
};

#define SYS_USB_DEVICES_DIR "/sys/class/usb_device"
#define VTUNER_PATH "/dev/misc"
#define MAX_ADAPTERS 8
#define BUFFER_SIZE ((188 / 4) * 4096) /* multiple of ts packet and page size */
#define DEMUX_BUFFER_SIZE (8 * ((188 / 4) * 4096)) /* 1.5MB */

#if DVB_API_VERSION < 5
typedef enum {
	DMX_TAP_TS = 0,
	DMX_TAP_PES = DMX_PES_OTHER, /* for backward binary compat. */
} dmx_tap_type_t;
#endif

struct vtuner_adapter
{
	int index;
	char name[64];
	int vtunerindex;
	char *buffer;
	int frontend;
	int demux;
	int vtuner;
	int firstdata;
	pthread_t eventthread, pumpthread;
	__u16 pidlist[30];
};

struct vtuner_adapter adapters[MAX_ADAPTERS];
int adaptercount = 0;
int vtunercount = 0;

int running = 0;
void sigint_handler(int sig)
{
	running = 1;
}

void sort_adapters()
{
	int i;
	for (i = adaptercount - 1; i > 0; i--)
	{
		int j;
		for (j = 0; j < i; j++)
		{
			if (adapters[j].index > adapters[j + 1].index)
			{
				char name[64];
				int index;
				strcpy(name, adapters[j].name);
				index = adapters[j].index;

				strcpy(adapters[j].name, adapters[j + 1].name);
				adapters[j].index = adapters[j + 1].index;

				strcpy(adapters[j + 1].name, name);
				adapters[j + 1].index = index;
			}
		}
	}
}

void scan_adapters()
{
	DIR *dirusb, *dirdev, *dirvtun;
	struct dirent *edirusb, *edirdev, *edirvtun;
	int i;

	/* adapters detect */
	dirusb = opendir(SYS_USB_DEVICES_DIR);
	if (!dirusb) return;

	while ((edirusb = readdir (dirusb)) != NULL && adaptercount < MAX_ADAPTERS)
	{
		char devdir[256];
		if (edirusb->d_name[0] == '.') continue;

		sprintf(devdir, "%s/%s/device", SYS_USB_DEVICES_DIR, edirusb->d_name);

		dirdev = opendir(devdir);
		if (!dirdev) continue;

		while ((edirdev = readdir (dirdev)) != NULL && adaptercount < MAX_ADAPTERS)
		{
			FILE *fd;
			char filename[256];

			if (strlen(edirdev->d_name) < 9) continue;
			if (strcmp(edirdev->d_name + (strlen(edirdev->d_name) - 9), "frontend0")) continue;

			sprintf(filename, "%s/%s/device/product", SYS_USB_DEVICES_DIR, edirusb->d_name);
			fd = fopen(filename, "r");
			if (!fd)
			{
				sprintf(filename, "%s/%s/device/manufacturer", SYS_USB_DEVICES_DIR, edirusb->d_name);
				fd = fopen(filename, "r");
			}

			if (fd)
			{
				char *tmp = adapters[adaptercount].name;
				fread(tmp, 63, 1, fd);
				tmp[63] = 0;
				while (strlen(tmp) > 0 && (tmp[strlen(tmp) - 1] == '\n' || tmp[strlen(tmp) - 1] == ' ')) tmp[strlen(tmp) - 1] = 0;
				fclose(fd);
			}
			else
			{
				strcpy(adapters[adaptercount].name, "unknown frontend");
			}

			adapters[adaptercount].index = edirdev->d_name[7] - '0';
			adaptercount++;
		}
		closedir(dirdev);
	}
	closedir(dirusb);

	dirvtun = opendir(VTUNER_PATH);
	if (dirvtun)
	{
		while ((edirvtun = readdir(dirvtun)) != NULL)
		{
			if (strlen(edirvtun->d_name) < 7) continue;
			if (!strncmp(edirvtun->d_name, "vtuner", 6)) vtunercount++;
		}
		closedir(dirvtun);
	}

	sort_adapters();

	for (i = 0; i < adaptercount; i++)
	{
		if (i < vtunercount)
		{
			adapters[i].vtunerindex = i;
			printf("usb device %s (adapter%d) assigned to vtuner%d\n", adapters[i].name, adapters[i].index, i);
		}
		else
		{
			adapters[i].vtunerindex = -1;
			printf("usb device %s (adapter%d) not assigned\n", adapters[i].name, adapters[i].index);
		}
	}
}


int _select(int maxfd, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout)
{
	int retval;
	fd_set rset, wset, xset;
	struct timeval interval;
	timerclear(&interval);

	/* make a backup of all fd_set's and timeval struct */
	if (readfds) rset = *readfds;
	if (writefds) wset = *writefds;
	if (exceptfds) xset = *exceptfds;
	if (timeout) interval = *timeout;

	while (1)
	{
		retval = select(maxfd, readfds, writefds, exceptfds, timeout);

		if (retval < 0)
		{
			/* restore the backup before we continue */
			if (readfds) *readfds = rset;
			if (writefds) *writefds = wset;
			if (exceptfds) *exceptfds = xset;
			if (timeout) *timeout = interval;
			if (errno == EINTR) continue;
			perror("select");
			break;
		}
		break;
	}
	return retval;
}

ssize_t _writeall(int fd, const void *buf, size_t count)
{
	ssize_t retval;
	char *ptr = (char*)buf;
	ssize_t handledcount = 0;
	if (fd < 0) return -1;
	while (handledcount < count)
	{
		retval = write(fd, &ptr[handledcount], count - handledcount);

		if (retval == 0) return -1;
		if (retval < 0)
		{
			if (errno == EINTR) continue;
			perror("write");
			return retval;
		}
		handledcount += retval;
	}
	return handledcount;
}

ssize_t _read(int fd, void *buf, size_t count)
{
	ssize_t retval;
	char *ptr = (char*)buf;
	ssize_t handledcount = 0;
	if (fd < 0) return -1;
	while (handledcount < count)
	{
		retval = read(fd, &ptr[handledcount], count - handledcount);
		if (retval < 0)
		{
			if (errno == EINTR) continue;
			perror("read");
			return retval;
		}
		handledcount += retval;
		break; /* one read only */
	}
	return handledcount;
}

void *pump_proc(void *ptr)
{
	struct vtuner_adapter *adapter = (struct vtuner_adapter *)ptr;

	while (!running)
	{
		struct timeval tv;
		fd_set rset;
		FD_ZERO(&rset);
		FD_SET(adapter->demux, &rset);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		if (_select(adapter->demux + 1, &rset, NULL, NULL, &tv) > 0)
		{
			int size = _read(adapter->demux, adapter->buffer, BUFFER_SIZE);
			if (_writeall(adapter->vtuner, adapter->buffer, size) <= 0)
			{
				break;
			}
		}
	}

	return NULL;
}

void *event_proc(void *ptr)
{
	int i, j;
	struct vtuner_adapter *adapter = (struct vtuner_adapter*)ptr;

	while (!running)
	{
		struct timeval tv;
		fd_set xset;
		FD_ZERO(&xset);
		FD_SET(adapter->vtuner, &xset);
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		if (_select(adapter->vtuner + 1, NULL, NULL, &xset, &tv) > 0)
		{
			struct vtuner_message message;
			ioctl(adapter->vtuner, VTUNER_GET_MESSAGE, &message);

			switch (message.type)
			{
			case MSG_SET_FRONTEND:
				adapter->firstdata = 1;
				ioctl(adapter->frontend, FE_SET_FRONTEND, &message.body.dvb_frontend_parameters);
				break;
			case MSG_GET_FRONTEND:
				ioctl(adapter->frontend, FE_GET_FRONTEND, &message.body.dvb_frontend_parameters);
				break;
			case MSG_READ_STATUS:
				ioctl(adapter->frontend, FE_READ_STATUS, &message.body.status);
				break;
			case MSG_READ_BER:
				ioctl(adapter->frontend, FE_READ_BER, &message.body.ber);
				break;
			case MSG_READ_SIGNAL_STRENGTH:
				ioctl(adapter->frontend, FE_READ_SIGNAL_STRENGTH, &message.body.ss);
				break;
			case MSG_READ_SNR:
				ioctl(adapter->frontend, FE_READ_SNR, &message.body.snr);
				break;
			case MSG_READ_UCBLOCKS:
				ioctl(adapter->frontend, FE_READ_UNCORRECTED_BLOCKS, &message.body.ucb);
				break;
			case MSG_SET_TONE:
				ioctl(adapter->frontend, FE_SET_TONE, &message.body.tone);
				break;
			case MSG_SEND_DISEQC_MSG:
				ioctl(adapter->frontend, FE_DISEQC_SEND_MASTER_CMD, &message.body.pad);
				break;
			case MSG_SEND_DISEQC_BURST:
				ioctl(adapter->frontend, FE_DISEQC_SEND_BURST, &message.body.pad);
				break;
			case MSG_PIDLIST:
				/* remove old pids */
				for (i = 0; i < 30; i++)
				{
					int found = 0;
					for (j = 0; j < 30; j++)
					{
						if (adapter->pidlist[i] == message.body.pidlist[j])
						{
							found = 1;
							break;
						}
					}

					if (found) continue;

					if (adapter->pidlist[i] != 0xffff)
					{
						printf("DMX_REMOVE_PID %x\n", adapter->pidlist[i]);
#if DVB_API_VERSION > 3
						ioctl(adapter->demux, DMX_REMOVE_PID, &adapter->pidlist[i]);
#else
						ioctl(adapter->demux, DMX_REMOVE_PID, adapter->pidlist[i]);
#endif
					}
				}

				/* add new pids */
				for (i = 0; i < 30; i++)
				{
					int found = 0;
					for (j = 0; j < 30; j++)
					{
						if (message.body.pidlist[i] == adapter->pidlist[j])
						{
							found = 1;
							break;
						}
					}

					if (found) continue;

					if (message.body.pidlist[i] != 0xffff)
					{
						printf("DMX_ADD_PID %x\n", message.body.pidlist[i]);
#if DVB_API_VERSION > 3
						ioctl(adapter->demux, DMX_ADD_PID, &message.body.pidlist[i]);
#else
						ioctl(adapter->demux, DMX_ADD_PID, message.body.pidlist[i]);
#endif
					}
				}

				/* copy pids */
				for (i = 0; i < 30; i++)
				{
					adapter->pidlist[i] = message.body.pidlist[i];
				}
				break;
			case MSG_SET_VOLTAGE:
			case MSG_ENABLE_HIGH_VOLTAGE:
				break;

			default:
				printf("Unknown vtuner message type: %d\n", message.type);
				break;
			}

			if (message.type != MSG_PIDLIST)
			{
				message.type = 0;
				ioctl(adapter->vtuner, VTUNER_SET_RESPONSE, &message);
			}
		}
	}

error:
	return NULL;
}

int init_adapter(int id)
{
	char type[8];
	struct dmx_pes_filter_params filter;
	struct dvb_frontend_info fe_info;
	char frontend_filename[256], demux_filename[256], vtuner_filename[256];

	struct vtuner_adapter *adapter = &adapters[id];

	adapter->eventthread = 0;
	adapter->pumpthread = 0;

	printf("linking adapter%d/frontend0 to vtuner%d\n", adapter->index, adapter->vtunerindex);

	sprintf(frontend_filename, "/dev/dvb/adapter%d/frontend0", adapter->index);
	sprintf(demux_filename, "/dev/dvb/adapter%d/demux0", adapter->index);
	sprintf(vtuner_filename, "/dev/misc/vtuner%d", adapter->vtunerindex);

	adapter->frontend = adapter->demux = adapter->vtuner = -1;

	adapter->frontend = open(frontend_filename, O_RDWR);
	if (adapter->frontend < 0)
	{
		perror(frontend_filename);
		goto error;
	}

	adapter->demux = open(demux_filename, O_RDONLY | O_NONBLOCK);
	if (adapter->demux < 0)
	{
		perror(demux_filename);
		goto error;
	}

	adapter->vtuner = open(vtuner_filename, O_RDWR);
	if (adapter->vtuner < 0)
	{
		perror(vtuner_filename);
		goto error;
	}

	if (ioctl(adapter->frontend, FE_GET_INFO, &fe_info) < 0)
	{
		perror("FE_GET_INFO");
		goto error;
	}

	filter.input = DMX_IN_FRONTEND;
	filter.flags = 0;
#if DVB_API_VERSION > 3
	filter.pid = 0;
	filter.output = DMX_OUT_TSDEMUX_TAP;
	filter.pes_type = DMX_PES_OTHER;
#else
	filter.pid = -1;
	filter.output = DMX_OUT_TAP;
	filter.pes_type = DMX_TAP_TS;
#endif

	ioctl(adapter->demux, DMX_SET_BUFFER_SIZE, DEMUX_BUFFER_SIZE);
	ioctl(adapter->demux, DMX_SET_PES_FILTER, &filter);
	ioctl(adapter->demux, DMX_START);

	switch (fe_info.type)
	{
	case FE_QPSK:
		strcpy(type,"DVB-S2");
		break;
	case FE_QAM:
		strcpy(type,"DVB-C");
		break;
	case FE_OFDM:
		strcpy(type,"DVB-T");
		break;
	default:
		printf("Frontend type 0x%x not supported", fe_info.type);
		goto error;
	}

	ioctl(adapter->vtuner, VTUNER_SET_NAME, adapter->name);
	ioctl(adapter->vtuner, VTUNER_SET_TYPE, type);
	ioctl(adapter->vtuner, VTUNER_SET_FE_INFO, &fe_info);
	ioctl(adapter->vtuner, VTUNER_SET_HAS_OUTPUTS, "no");

	memset(adapter->pidlist, 0xff, sizeof(adapter->pidlist));
	adapter->buffer = malloc(BUFFER_SIZE);

	printf("init succeeded\n");
	pthread_create(&adapter->eventthread, NULL, event_proc, (void*)adapter);
	pthread_create(&adapter->pumpthread, NULL, pump_proc, (void*)adapter);
	return 0;

error:
	if (adapter->vtuner >= 0)
	{
		close(adapter->vtuner);
		adapter->vtuner = -1;
	}
	if (adapter->demux >= 0)
	{
		close(adapter->demux);
		adapter->demux = -1;
	}
	if (adapter->frontend >= 0)
	{
		close(adapter->frontend);
		adapter->frontend = -1;
	}
	printf("init failed\n");
	return -1;
}

void daemon_init()
{
	pid_t pid;

	if ((pid = fork()) == 0) _exit(EXIT_SUCCESS);

	setsid(); /* become session leader */

	umask(0);

	/* forks, chdirs to /, closes files */
	daemon(0, 0);

	signal(SIGHUP, SIG_IGN);
	signal(SIGPIPE, SIG_IGN);
}

int main(int argc, char *argv[])
{
	struct sigaction action;
	int i;
	int ok = 0;
	int debug = 0;

	char option;
	while ((option = getopt(argc, argv, "dh")) >= 0)
	{
		switch (option)
		{
		case 'd':
			debug = 1;
			break;
		default:
			printf("%s [-d]\n\t-d: do not daemonize\n", argv[0]);
			break;
		}
	}

	if (!debug) daemon_init();

	signal(SIGTERM, sigint_handler);
	signal(SIGINT, sigint_handler);

	scan_adapters();

	for (i = 0; i < adaptercount; i++)
	{
		if (adapters[i].vtunerindex >= 0) init_adapter(i);
	}

	for (i = 0; i < adaptercount; i++)
	{
		if (adapters[i].vtunerindex >= 0)
		{
			ok = 1;
			if (adapters[i].eventthread && adapters[i].pumpthread)
			{
				pthread_join(adapters[i].eventthread, NULL);
				pthread_join(adapters[i].pumpthread, NULL);

				free(adapters[i].buffer);
				close(adapters[i].vtuner);
				close(adapters[i].demux);
				close(adapters[i].frontend);
			}
		}
	}

	exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}
