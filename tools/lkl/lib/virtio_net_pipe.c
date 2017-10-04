/*
 * POSIX file descriptor(in and out) based
 * virtual network interface feature for LKL
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/uio.h>

#include "virtio.h"

struct lkl_netdev_pipe {
	struct lkl_netdev dev;
	/* file-descriptor based device */
	int fd_rx;
	int fd_tx;
	/*
	 * Controlls the poll mask for fd. Can be acccessed concurrently from
	 * poll, tx, or rx routines but there is no need for syncronization
	 * because:
	 *
	 * (a) TX and RX routines set different variables so even if they update
	 * at the same time there is no race condition
	 *
	 * (b) Even if poll and TX / RX update at the same time poll cannot
	 * stall: when poll resets the poll variable we know that TX / RX will
	 * run which means that eventually the poll variable will be set.
	 */
	int poll_tx, poll_rx;
	/* controle pipe */
	int pipe[2];
};

static int pipe_net_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);

	do {
		ret = writev(nd_pipe->fd_tx, iov, cnt);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to fd netdev fails");
		} else {
			char tmp;

			nd_pipe->poll_tx = 1;
			if (write(nd_pipe->pipe[1], &tmp, 1) <= 0)
				perror("virtio net fd pipe write");
		}
	}
	return ret;
}

static int pipe_net_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);

	do {
		ret = readv(nd_pipe->fd_rx, (struct iovec *)iov, cnt);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("virtio net fd read");
		} else {
			char tmp;

			nd_pipe->poll_rx = 1;
			if (write(nd_pipe->pipe[1], &tmp, 1) < 0)
				perror("virtio net fd pipe write");
		}
	}
	return ret;
}

static int pipe_net_poll(struct lkl_netdev *nd)
{
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);
	struct pollfd pfds[3] = {
		{
			.fd = nd_pipe->fd_rx,
		},
		{	.fd = nd_pipe->fd_tx,
		},
		{
			.fd = nd_pipe->pipe[0],
			.events = POLLIN,
		},
	};
	int ret;

	if (nd_pipe->poll_rx)
		pfds[0].events |= POLLIN|POLLPRI;
	if (nd_pipe->poll_tx)
		pfds[1].events |= POLLOUT;

	do {
		ret = poll(pfds, 3, -1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		perror("virtio net fd poll");
		return 0;
	}

	if (pfds[2].revents & (POLLHUP|POLLNVAL))
		return LKL_DEV_NET_POLL_HUP;

	if (pfds[2].revents & POLLIN) {
		char tmp[PIPE_BUF];

		ret = read(nd_pipe->pipe[0], tmp, PIPE_BUF);
		if (ret == 0)
			return LKL_DEV_NET_POLL_HUP;
		if (ret < 0)
			perror("virtio net fd pipe read");
	}

	ret = 0;

	if (pfds[0].revents & (POLLIN|POLLPRI)) {
		nd_pipe->poll_rx = 0;
		ret |= LKL_DEV_NET_POLL_RX;
	}

	if (pfds[1].revents & POLLOUT) {
		nd_pipe->poll_tx = 0;
		ret |= LKL_DEV_NET_POLL_TX;
	}

	return ret;
}

static void pipe_net_poll_hup(struct lkl_netdev *nd)
{
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);

	/* this will cause a POLLHUP / POLLNVAL in the poll function */
	close(nd_pipe->pipe[0]);
	close(nd_pipe->pipe[1]);
}

static void pipe_net_free(struct lkl_netdev *nd)
{
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);

	close(nd_pipe->fd_rx);
	close(nd_pipe->fd_tx);
	free(nd_pipe);
}

struct lkl_dev_net_ops pipe_net_ops =  {
	.tx = pipe_net_tx,
	.rx = pipe_net_rx,
	.poll = pipe_net_poll,
	.poll_hup = pipe_net_poll_hup,
	.free = pipe_net_free,
};

struct lkl_netdev *lkl_register_netdev_pipe(int fd_rx, int fd_tx)
{
	struct lkl_netdev_pipe *nd;

	nd = malloc(sizeof(*nd));
	if (!nd) {
		fprintf(stderr, "fdnet: failed to allocate memory\n");
		/* TODO: propagate the error state, maybe use errno for that? */
		return NULL;
	}

	memset(nd, 0, sizeof(*nd));

	nd->fd_rx = fd_rx;
	nd->fd_tx = fd_tx;
	if (pipe(nd->pipe) < 0) {
		perror("pipe");
		free(nd);
		return NULL;
	}

	if (fcntl(nd->pipe[0], F_SETFL, O_NONBLOCK) < 0) {
		perror("fnctl");
		close(nd->pipe[0]);
		close(nd->pipe[1]);
		free(nd);
		return NULL;
	}

	nd->dev.ops = &pipe_net_ops;
	return &nd->dev;
}

#define BIT(x) (1ULL << x)

struct lkl_netdev *lkl_netdev_pipe_create(char *ifname, int offload)
{
	struct lkl_netdev *nd;
	int fd_rx, fd_tx;

	char *ifname_rx = NULL, *ifname_tx = NULL;

	ifname_rx = strtok(ifname, "|");
	if(ifname_rx == NULL){
		fprintf(stderr, "invalid ifname format\n");
		return NULL;
	}

	ifname_tx = strtok(NULL, "|");
	if(ifname_tx == NULL){
		fprintf(stderr, "invalid ifname format\n");
		return NULL;
	}

	if(strtok(NULL, "|") != NULL){
		fprintf(stderr, "invalid ifname format\n");
		return NULL;
	}

	fd_rx = open(ifname_rx, O_RDWR|O_NONBLOCK);
	if (fd_rx < 0) {
		perror("can not open ifname_rx pipe");
		return NULL;
	}

	fd_tx = open(ifname_tx, O_RDWR|O_NONBLOCK);
	if (fd_tx < 0) {
		perror("can not open ifname_tx pipe");
		return NULL;
	}

	nd = lkl_register_netdev_pipe(fd_rx, fd_tx);
	if (!nd) {
		perror("failed to register to.");
		close(fd_rx);
		close(fd_tx);
		return NULL;
	}

	/* 
	 * Pipe does not do anything for offload
	 * assume counter of pipe also has offload
	 * and enable vnet_hdr
	 * */
	if (offload & (BIT(LKL_VIRTIO_NET_F_GUEST_CSUM)   |
			BIT(LKL_VIRTIO_NET_F_GUEST_TSO4)  |
			BIT(LKL_VIRTIO_NET_F_MRG_RXBUF)   |
			BIT(LKL_VIRTIO_NET_F_GUEST_TSO6) |
			BIT(LKL_VIRTIO_NET_F_CSUM) |
			BIT(LKL_VIRTIO_NET_F_HOST_TSO4) |
			BIT(LKL_VIRTIO_NET_F_HOST_TSO6))){
		nd->has_vnet_hdr = 1;
	}

	return nd;
}
