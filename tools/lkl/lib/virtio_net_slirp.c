/*
 * slirp based virtual network interface feature for LKL
 * Copyright (c) 2017,2016 Motomu Utsumi
 *
 * Author: Motomu Utsumi <motomuman@gmail.com>
 *
 * Current implementation is linux-specific.
 */

#define _GNU_SOURCE
#include <termios.h>
#include <stdlib.h>
#include <sys/un.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#ifdef __FreeBSD__
#include <sys/syslimits.h>
#else
#include <limits.h>
#endif
#include <fcntl.h>
#include <sys/poll.h>

#include "virtio.h"

#define SLIRP_BUF_SIZE 5000

/* SLIP protocol characters. */
#define SLIP_END             0300	/* indicates end of frame	*/
#define SLIP_ESC             0333	/* indicates byte stuffing	*/
#define SLIP_ESC_END         0334	/* ESC ESC_END means END 'data'	*/
#define SLIP_ESC_ESC         0335	/* ESC ESC_ESC means ESC 'data'	*/

struct lkl_netdev_slirp {
	struct lkl_netdev dev;
	/* file-descriptor based device */
	int slirp_rx;
	int slirp_tx;
	/* buffer for slirp */
	int ipos;
	uint8_t ibuf[SLIRP_BUF_SIZE];
	uint8_t obuf[SLIRP_BUF_SIZE];
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

static int buf_has_pkt(uint8_t *buf, int len)
{
	int i;

	for (i = 1; i < len; i++)
		if (buf[i] == SLIP_END)
			return 1;
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

static uint8_t *iov_head(struct iovec *iov, int *cnt)
{
	for (; (*cnt) > 0; (*cnt)--) {
		if (iov[0].iov_len != 0)
			return iov[0].iov_base;
	}
	return NULL;
}

static uint8_t *iov_next(struct iovec *iov, int *cnt)
{
	iov[0].iov_base++;
	iov[0].iov_len--;
	return iov_head(iov, cnt);
}

static int check_skip_ethhdr(struct iovec *iov, int *cnt)
{
	int i;
	uint8_t *iovptr;

	iovptr = iov_head(iov, cnt);
	if (iovptr == NULL)
		return -1;

	for (i = 0; i < LKL_ETH_HLEN; i++) {
		if (i == 12 && *iovptr != 0x08)
			return -1;
		if (i == 13 && *iovptr != 0x00)
			return -1;

		iovptr = iov_next(iov, cnt);
		if (iovptr == NULL)
			return -1;
	}
	return 0;
}

static int set_ethhdr(struct lkl_netdev *nd, struct iovec *iov, int *cnt)
{
	int i;
	uint8_t *iovptr;

	iovptr = iov_head(iov, cnt);
	if (iovptr == NULL)
		return -1;

	/*
	 * Set destination mac
	 */
	for (i = 0; i < LKL_ETH_ALEN; i++) {
		*iovptr = nd->mac[i];
		iovptr = iov_next(iov, cnt);
		if (iovptr == NULL)
			return -1;
	}

	/*
	 * Skip source mac
	 */
	for (i = 0; i < LKL_ETH_ALEN; i++) {
		iovptr = iov_next(iov, cnt);
		if (iovptr == NULL)
			return -1;
	}

	/*
	 * Set IPv4 Type
	 */
	*iovptr = 0x08;
	iovptr = iov_next(iov, cnt);
	if (iovptr == NULL)
		return -1;
	*iovptr = 0x00;
	iovptr = iov_next(iov, cnt);
	if (iovptr == NULL)
		return -1;
	return 0;
}

static int slirp_esc(struct lkl_netdev_slirp *nd_slirp,
		struct iovec *iov, int *cnt)
{
	uint8_t *dptr = nd_slirp->obuf;
	uint8_t *iovptr;

	iovptr = iov_head(iov, cnt);
	if (iovptr == NULL)
		return -1;

	*dptr++ = SLIP_END;

	do {
		switch (*iovptr) {
		case SLIP_END:
			*dptr++ = SLIP_ESC;
			*dptr++ = SLIP_ESC_END;
			break;
		case SLIP_ESC:
			*dptr++ = SLIP_ESC;
			*dptr++ = SLIP_ESC_ESC;
			break;
		default:
			*dptr++ = *iovptr;
			break;
		}
	} while ((iovptr = iov_next(iov, cnt)) != NULL &&
			(dptr - nd_slirp->obuf) < SLIRP_BUF_SIZE);

	if ((dptr - nd_slirp->obuf) >= SLIRP_BUF_SIZE) {
		fprintf(stderr, "slirp esc length error\n");
		return -1;
	}

	*dptr++ = SLIP_END;
	return (dptr - nd_slirp->obuf);
}

static int slirp_unesc(struct lkl_netdev_slirp *nd_slirp,
		struct iovec *iov, int *cnt)
{
	int i, length, esc;
	uint8_t c;
	uint8_t *iovptr;
	uint8_t *sbuf = nd_slirp->ibuf;

	length = 0;
	esc = 0;

	iovptr = iov_head(iov, cnt);
	if (iovptr == NULL)
		return -1;

	if (sbuf[0] != SLIP_END) {
		*iovptr = sbuf[0];
		length++;
		iovptr = iov_next(iov, cnt);
		if (iovptr == NULL)
			return -1;
	}

	for (i = 1; i < nd_slirp->ipos; i++) {
		c = sbuf[i];
		switch (c) {
		case SLIP_END:
			if (esc)
				fprintf(stderr,
					"slirp unesc invalid packet\n");
			i++;
			memmove(sbuf, sbuf + i, nd_slirp->ipos - i);
			nd_slirp->ipos -= i;
			return length;
		case SLIP_ESC:
			if (esc)
				fprintf(stderr,
					"slirp unesc invalid packet\n");
			esc = 1;
			break;
		case SLIP_ESC_ESC:
			if (esc) {
				esc = 0;
				*iovptr = SLIP_ESC;
				length++;
				iovptr = iov_next(iov, cnt);
				if (iovptr == NULL)
					return -1;
			} else {
				*iovptr = c;
				length++;
				iovptr = iov_next(iov, cnt);
				if (iovptr == NULL)
					return -1;
			}
			break;
		case SLIP_ESC_END:
			if (esc) {
				esc = 0;
				*iovptr = SLIP_END;
				length++;
				iovptr = iov_next(iov, cnt);
				if (iovptr == NULL)
					return -1;
			} else {
				*iovptr = c;
				length++;
				iovptr = iov_next(iov, cnt);
				if (iovptr == NULL)
					return -1;
			}
			break;
		default:
			if (esc)
				fprintf(stderr,
					"slirp unesc invalid packet\n");
			*iovptr = c;
			length++;
			iovptr = iov_next(iov, cnt);
			if (iovptr == NULL)
				return -1;
			break;
		}
	}
	fprintf(stderr, "BUG slirp unesc, does not reach SLIP_END\n");
	return -1;
}

static int slirp_net_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret, len, pktlen;
	struct lkl_netdev_slirp *nd_slirp =
		container_of(nd, struct lkl_netdev_slirp, dev);
	struct iovec work_iov[VIRTIO_REQ_MAX_BUFS];

	if (cnt > VIRTIO_REQ_MAX_BUFS) {
		fprintf(stderr, "tx iov array length error\n");
		return 0;
	}

	pktlen = get_iovlen(iov, cnt);

	copy_iov_info(iov, work_iov, cnt);

	ret = check_skip_ethhdr(work_iov, &cnt);
	if (ret < 0) {
		lkl_printf("Slirp backend support only IPv4 packets\n");
		return pktlen;
	}

	len = slirp_esc(nd_slirp, work_iov, &cnt);

	do {
		ret = write(nd_slirp->slirp_tx, nd_slirp->obuf, len);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to slirp netdev fails");
		} else {
			char tmp = 0;

			nd_slirp->poll_tx = 1;
			if (write(nd_slirp->pipe[1], &tmp, 1) <= 0)
				perror("virtio net slirp pipe write");
		}
	}
	return ret;
}

static int slirp_net_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_slirp *nd_slirp =
		container_of(nd, struct lkl_netdev_slirp, dev);
	struct iovec work_iov[VIRTIO_REQ_MAX_BUFS];

	if (cnt > VIRTIO_REQ_MAX_BUFS) {
		fprintf(stderr, "rx iov array length error\n");
		return 0;
	}

	if (!buf_has_pkt(nd_slirp->ibuf, nd_slirp->ipos)) {
		do {
			ret = read(nd_slirp->slirp_rx,
				nd_slirp->ibuf + nd_slirp->ipos,
				SLIRP_BUF_SIZE - nd_slirp->ipos);
		} while (ret == -1 && errno == EINTR);

		if (ret < 0) {
			if (errno != EAGAIN) {
				perror("virtio net slirp read");
			} else {
				char tmp = 0;

				nd_slirp->poll_rx = 1;
				if (write(nd_slirp->pipe[1], &tmp, 1) < 0)
					perror("virtio net slirp pipe write");
			}
			return ret;
		}

		nd_slirp->ipos += ret;
	}

	if (buf_has_pkt(nd_slirp->ibuf, nd_slirp->ipos)) {
		copy_iov_info(iov, work_iov, cnt);
		set_ethhdr(nd, work_iov, &cnt);
		ret = slirp_unesc(nd_slirp, work_iov, &cnt);
		if (ret < 0)
			return ret;
		return ret + LKL_ETH_HLEN;
	}

	return 0;
}

static int slirp_net_poll(struct lkl_netdev *nd)
{
	struct lkl_netdev_slirp *nd_slirp =
		container_of(nd, struct lkl_netdev_slirp, dev);
	struct pollfd pfds[3] = {
		{
			.fd = nd_slirp->slirp_rx,
		},
		{
			.fd = nd_slirp->slirp_tx,
		},
		{
			.fd = nd_slirp->pipe[0],
			.events = POLLIN,
		},
	};
	int ret;

	if (nd_slirp->poll_rx)
		pfds[0].events |= POLLIN|POLLPRI;
	if (nd_slirp->poll_tx)
		pfds[1].events |= POLLOUT;

	do {
		ret = poll(pfds, 3, -1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		perror("virtio net slirp poll");
		return 0;
	}

	if (pfds[2].revents & (POLLHUP|POLLNVAL))
		return LKL_DEV_NET_POLL_HUP;

	if (pfds[2].revents & POLLIN) {
		char tmp[PIPE_BUF];

		ret = read(nd_slirp->pipe[0], tmp, PIPE_BUF);
		if (ret == 0)
			return LKL_DEV_NET_POLL_HUP;
		if (ret < 0)
			perror("virtio net slirp pipe read");
	}

	ret = 0;

	if (pfds[0].revents & (POLLIN|POLLPRI)) {
		nd_slirp->poll_rx = 0;
		ret |= LKL_DEV_NET_POLL_RX;
	}

	if (pfds[1].revents & POLLOUT) {
		nd_slirp->poll_tx = 0;
		ret |= LKL_DEV_NET_POLL_TX;
	}

	return ret;
}

static void slirp_net_poll_hup(struct lkl_netdev *nd)
{
	struct lkl_netdev_slirp *nd_slirp =
		container_of(nd, struct lkl_netdev_slirp, dev);

	/* this will cause a POLLHUP / POLLNVAL in the poll function */
	close(nd_slirp->pipe[0]);
	close(nd_slirp->pipe[1]);
}

static void
ignore_signal()
{
}

static void slirp_net_free(struct lkl_netdev *nd)
{
	struct lkl_netdev_slirp *nd_slirp =
		container_of(nd, struct lkl_netdev_slirp, dev);
	struct sigaction act;

	/*
	 * Ignore sigquit from slirp
	 * lkl will exit soon
	 */
	memset(&act, 0, sizeof(act));
	act.sa_handler = ignore_signal;
	if (sigaction(SIGQUIT, &act, NULL) < 0)
		perror("sigaction");
	close(nd_slirp->slirp_rx);
	close(nd_slirp->slirp_tx);
	free(nd_slirp);
}

struct lkl_dev_net_ops slirp_net_ops =  {
	.tx = slirp_net_tx,
	.rx = slirp_net_rx,
	.poll = slirp_net_poll,
	.poll_hup = slirp_net_poll_hup,
	.free = slirp_net_free,
};

struct lkl_netdev *lkl_netdev_slirp_create(const char *_path)
{
	struct lkl_netdev_slirp *nd;
	struct sockaddr_un sa = {0};
	struct termios ts;
	int slirp_fd, sock;
	int unit, pid, ret, fd;
	char *ptr;
	char buf[256];

	/*
	 * Create pseudoterminal for slirp
	 */
	slirp_fd = open("/dev/ptmx",  O_RDWR|O_NONBLOCK);
	if (slirp_fd < 0) {
		perror("open ptmx");
		return NULL;
	}

	if (grantpt(slirp_fd) == -1) {
		perror("grantpt");
		return NULL;
	}
	if (unlockpt(slirp_fd) == -1) {
		perror("unlockpt");
		return NULL;
	}

	ptr = ptsname(slirp_fd);
	if (ptr == NULL) {
		perror("ptsname");
		return NULL;
	}

	/*
	 * Open slirp socket and tell
	 * pseudoterminal name
	 */
	sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sock == -1) {
		perror("socket");
		return NULL;
	}

	sa.sun_family = AF_UNIX;
	strcpy(sa.sun_path, _path);
	if (connect(sock, (struct sockaddr *)&sa,
				sizeof(struct sockaddr_un)) == -1) {
		perror("connect");
		return NULL;
	}

	unit = 0;
	pid = getpid();
	snprintf(buf, sizeof(buf), "%d %d %s", unit, pid, ptr);
	ret = write(sock, buf, sizeof(buf));
	ret = read(sock, buf, sizeof(buf));
	if (ret < 0 || buf[0] != '1') {
		lkl_printf("LKL may failed to attach slirp: %s\n",
				buf);
	}
	close(sock);

	fd = open(ptr, O_RDWR);
	if (fd < 0) {
		perror("open pts");
		return NULL;
	}
	if (tcgetattr(fd, &ts) < 0) {
		close(fd);
		perror("tcgetattr");
		return NULL;
	}
	cfmakeraw(&ts);
	if (tcsetattr(fd, TCSANOW, &ts) < 0) {
		close(fd);
		perror("tcsetattr");
		return NULL;
	}

	close(fd);

	nd = malloc(sizeof(*nd));
	if (!nd) {
		fprintf(stderr, "slirpnet: failed to allocate memory\n");
		/* TODO: propagate the error state, maybe use errno for that? */
		return NULL;
	}
	memset(nd, 0, sizeof(*nd));

	nd->slirp_rx = slirp_fd;
	nd->slirp_tx = slirp_fd;
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

	nd->dev.ops = &slirp_net_ops;
	nd->dev.is_slirp = 1;
	return &nd->dev;
}
