/*
 * pipe based virtual network interface feature for LKL
 * Copyright (c) 2017,2016 Motomu Utsumi
 *
 * Author: Motomu Utsumi <motomuman@gmail.com>
 *
 * Current implementation is linux-specific.
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#ifdef __FreeBSD__
#include <sys/syslimits.h>
#else
#include <limits.h>
#endif
#include <fcntl.h>
#include <sys/poll.h>

#include "virtio.h"

#define PIPE_PKT_HEADER_LEN 4

struct lkl_netdev_pipe {
	struct lkl_netdev dev;
	/* file-descriptor based device */
	int pipe_rx;
	int pipe_tx;
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

static int get_iovlen(struct iovec *iov, int cnt)
{
	int i, iovlen = 0;

	for (i = 0; i < cnt; i++)
		iovlen += iov[i].iov_len;
	return iovlen;
}

static int tx_set_next(int shift_len, struct iovec *iov, int cnt)
{
	int i, shift, cur = 0;

	for (i = 0; i < cnt; i++) {
		if (cur + (int)iov[i].iov_len <= shift_len)
			shift = iov[i].iov_len;
		else
			shift = shift_len - cur;
		iov[i].iov_base += shift;
		iov[i].iov_len -= shift;
		cur += shift;
		if (cur == shift_len)
			return 0;
	}
	fprintf(stderr, "BUG: tx set next shift(%d) exceeds iov length\n",
			shift_len);
	return -1;
}

static int write_next_pktlen(struct lkl_netdev_pipe *nd_pipe, int pktlen)
{
	int ret;
	struct iovec txiov;
	uint8_t tmpbuf[PIPE_PKT_HEADER_LEN];

	txiov.iov_len = PIPE_PKT_HEADER_LEN;
	tmpbuf[0] = (pktlen) & 0xff;
	tmpbuf[1] = (pktlen >> 8) & 0xff;
	tmpbuf[2] = (pktlen >> 16) & 0xff;
	tmpbuf[3] = (pktlen >> 24) & 0xff;
	txiov.iov_base = tmpbuf;

	do {
		ret = writev(nd_pipe->pipe_tx, &txiov, 1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to pipe netdev fails header");
		} else {
			char tmp = 0;

			nd_pipe->poll_tx = 1;
			if (write(nd_pipe->pipe[1], &tmp, 1) <= 0)
				perror("virtio net pipe pipe write");
		}
	}
	return ret;
}

static int wait_tx_pollout(int txfd)
{
	struct pollfd pfds[1] = {
		{
			.fd = txfd,
			.events = POLLOUT,
		},
	};

	poll(pfds, 1, -1);
	return 0;
}

static int copy_iov_info(struct iovec *iov, struct iovec *work_iov, int cnt)
{
	int i;

	for (i = 0; i < cnt; i++) {
		work_iov[i].iov_base = iov[i].iov_base;
		work_iov[i].iov_len = iov[i].iov_len;
	}
	return 0;
}

static int pipe_net_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);
	int ret, pktlen, write_len = 0;
	struct iovec work_iov[VIRTIO_REQ_MAX_BUFS];

	if (cnt > VIRTIO_REQ_MAX_BUFS) {
		fprintf(stderr, "rx iov array length exceeds\n");
		return 0;
	}

	pktlen = get_iovlen(iov, cnt);
	ret = write_next_pktlen(nd_pipe, pktlen);
	if (ret < 0)
		return ret;

	copy_iov_info(iov, work_iov, cnt);

	while (write_len < pktlen) {
		do {
			ret = writev(nd_pipe->pipe_tx, work_iov, cnt);
		} while (ret == -1 && errno == EINTR);

		if (ret < 0) {
			if (errno != EAGAIN)
				perror("write to pipe netdev fails");
			wait_tx_pollout(nd_pipe->pipe_tx);
		} else {
			write_len += ret;
			if (write_len == pktlen)
				break;
			tx_set_next(ret, work_iov, cnt);
		}
	}
	return pktlen;
}

static int rx_adjust_len(int pktlen, struct iovec *iov, int cnt)
{
	int i, adjust, cur_len = 0;

	for (i = 0; i < cnt; i++) {
		if (cur_len + (int)iov[i].iov_len > pktlen)
			adjust = iov[i].iov_len - pktlen + cur_len;
		else
			adjust = 0;
		iov[i].iov_len -= adjust;
		cur_len += iov[i].iov_len;
	}
	if (cur_len < pktlen) {
		fprintf(stderr, "BUG: rx adjust length error pktlen = %d\n",
				pktlen);
		return -1;
	}
	return 0;
}

static int rx_set_next(int shift_len, struct iovec *iov, int cnt)
{
	int i, shift, cur_len = 0;

	for (i = 0; i < cnt && cur_len < shift_len; i++) {
		if (cur_len + (int)iov[i].iov_len >= shift_len)
			shift = shift_len - cur_len;
		else
			shift = iov[i].iov_len;
		cur_len += shift;
		iov[i].iov_base += shift;
		iov[i].iov_len -= shift;
	}

	if (cur_len < shift_len) {
		fprintf(stderr, "BUG: rx set next iov, shift length %d\n",
				shift_len);
		return -1;
	}

	return 0;
}

static int read_next_pktlen(struct lkl_netdev_pipe *nd_pipe)
{
	int ret, pktlen;
	struct iovec rxiov;
	uint8_t tmpbuf[PIPE_PKT_HEADER_LEN];

	rxiov.iov_len = PIPE_PKT_HEADER_LEN;
	rxiov.iov_base = tmpbuf;
	do {
		ret = readv(nd_pipe->pipe_rx, (struct iovec *)&rxiov, 1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("virtio net pipe read");
		} else {
			char tmp = 0;

			nd_pipe->poll_rx = 1;
			if (write(nd_pipe->pipe[1], &tmp, 1) < 0)
				perror("virtio net pipe pipe write");
		}
		return ret;
	}
	pktlen =  ((uint8_t *)rxiov.iov_base)[0];
	pktlen += (((uint8_t *)rxiov.iov_base)[1]<<8);
	pktlen += (((uint8_t *)rxiov.iov_base)[2]<<16);
	pktlen += (((uint8_t *)rxiov.iov_base)[3]<<24);
	return pktlen;
}

static int wait_rx_pollin(int rxfd)
{
	struct pollfd pfds[1] = {
		{
			.fd = rxfd,
			.events = POLLIN|POLLPRI,
		},
	};

	poll(pfds, 1, -1);
	return 0;
}

static int pipe_net_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);
	int read_len = 0;
	int ret, iovlen, pktlen;
	struct iovec work_iov[VIRTIO_REQ_MAX_BUFS];

	if (cnt > VIRTIO_REQ_MAX_BUFS) {
		fprintf(stderr, "rx iov array length exceeds\n");
		return 0;
	}

	ret = read_next_pktlen(nd_pipe);
	if (ret < 0)
		return ret;
	pktlen = ret;
	iovlen = get_iovlen(iov, cnt);
	copy_iov_info(iov, work_iov, cnt);

	if (pktlen > iovlen) {
		/*
		 * discard packet
		 * maybe should not use iov
		 */
		while (pktlen) {
			if (get_iovlen(work_iov, cnt) > pktlen)
				rx_adjust_len(pktlen, work_iov, cnt);

			do {
				ret = readv(nd_pipe->pipe_rx,
						(struct iovec *)work_iov, cnt);
			} while (ret == -1 && errno == EINTR);

			if (ret < 0) {
				if (errno != EAGAIN)
					perror("read from pipe netdev fails");
				wait_rx_pollin(nd_pipe->pipe_rx);
			} else {
				pktlen -= ret;
			}
		}

		return iovlen;
	}

	rx_adjust_len(pktlen, work_iov, cnt);
	while (read_len < pktlen) {
		do {
			ret = readv(nd_pipe->pipe_rx,
					(struct iovec *)work_iov, cnt);
		} while (ret == -1 && errno == EINTR);

		if (ret < 0) {
			if (errno != EAGAIN)
				perror("write to pipe netdev fails");
			wait_rx_pollin(nd_pipe->pipe_rx);
		} else {
			read_len += ret;
			if (read_len == pktlen)
				break;
			rx_set_next(ret, work_iov, cnt);
		}
	}

	return pktlen;
}

static int pipe_net_poll(struct lkl_netdev *nd)
{
	struct lkl_netdev_pipe *nd_pipe =
		container_of(nd, struct lkl_netdev_pipe, dev);
	struct pollfd pfds[3] = {
		{
			.fd = nd_pipe->pipe_rx,
		},
		{
			.fd = nd_pipe->pipe_tx,
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
		perror("virtio net pipe poll");
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
			perror("virtio net pipe pipe read");
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

	close(nd_pipe->pipe_rx);
	close(nd_pipe->pipe_tx);
	free(nd_pipe);
}

struct lkl_dev_net_ops pipe_net_ops =  {
	.tx = pipe_net_tx,
	.rx = pipe_net_rx,
	.poll = pipe_net_poll,
	.poll_hup = pipe_net_poll_hup,
	.free = pipe_net_free,
};

struct lkl_netdev *lkl_netdev_pipe_create(const char *_ifname, int offload)
{
	struct lkl_netdev_pipe *nd;
	int fd_rx, fd_tx;
	char *ifname = strdup(_ifname), *ifname_rx = NULL, *ifname_tx = NULL;

	ifname_rx = strtok(ifname, "|");
	if (ifname_rx == NULL) {
		fprintf(stderr, "invalid ifname format: %s\n", ifname);
		free(ifname);
		return NULL;
	}

	ifname_tx = strtok(NULL, "|");
	if (ifname_tx == NULL) {
		fprintf(stderr, "invalid ifname format: %s\n", ifname);
		free(ifname);
		return NULL;
	}

	if (strtok(NULL, "|") != NULL) {
		fprintf(stderr, "invalid ifname format: %s\n", ifname);
		free(ifname);
		return NULL;
	}

	fd_rx = open(ifname_rx, O_RDWR|O_NONBLOCK);
	if (fd_rx < 0) {
		perror("can not open ifname_rx pipe");
		free(ifname);
		return NULL;
	}

	fd_tx = open(ifname_tx, O_RDWR|O_NONBLOCK);
	if (fd_tx < 0) {
		perror("can not open ifname_tx pipe");
		close(fd_rx);
		free(ifname);
		return NULL;
	}

	nd = malloc(sizeof(*nd));
	if (!nd) {
		fprintf(stderr, "pipenet: failed to allocate memory\n");
		close(fd_rx);
		close(fd_tx);
		free(ifname);
		/* TODO: propagate the error state, maybe use errno for that? */
		return NULL;
	}

	memset(nd, 0, sizeof(*nd));

	nd->pipe_rx = fd_rx;
	nd->pipe_tx = fd_tx;
	if (pipe(nd->pipe) < 0) {
		perror("pipe");
		close(fd_rx);
		close(fd_tx);
		free(ifname);
		free(nd);
		return NULL;
	}

	if (fcntl(nd->pipe[0], F_SETFL, O_NONBLOCK) < 0) {
		perror("fnctl");
		close(fd_rx);
		close(fd_tx);
		close(nd->pipe[0]);
		close(nd->pipe[1]);
		free(ifname);
		free(nd);
		return NULL;
	}

	nd->dev.ops = &pipe_net_ops;
	free(ifname);
	/*
	 * To avoid mismatch with LKL otherside,
	 * we always enabled vnet hdr
	 */
	nd->dev.has_vnet_hdr = 1;
	return &nd->dev;
}
