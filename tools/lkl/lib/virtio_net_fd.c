/*
 * POSIX file descriptor based virtual network interface feature for
 * LKL Copyright (c) 2015,2016 Ryo Nakamura, Hajime Tazaki
 *
 * Author: Ryo Nakamura <upa@wide.ad.jp>
 *         Hajime Tazaki <thehajime@gmail.com>
 *         Octavian Purdila <octavian.purdila@intel.com>
 *
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
#include <sys/uio.h>

#include "virtio.h"
#include "virtio_net_fd.h"

#define TX_HEADER_LEN 4

struct lkl_netdev_fd {
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
	/* pipe device */
	int is_pipe :1;
};

static int get_iovlen(struct iovec *iov, int cnt)
{
	int iovlen = 0;
	int i;

	for(i = 0; i < cnt; i++){
		iovlen += iov[i].iov_len;
	}
	return iovlen;
}

static int iov_ptr_restore(int iov_ptr_move[], struct iovec *iov, int cnt)
{
	int i;
	for(i = 0; i < cnt; i++) {
		iov[i].iov_base += iov_ptr_move[i];
	}
	return 0;
}

static int iov_len_restore(int iov_len_move[], struct iovec *iov, int cnt)
{
	int i;
	for(i = 0; i < cnt; i++) {
		iov[i].iov_len += iov_len_move[i];
	}
	return 0;
}

static void iov_move_ptr(int shift_len, int iov_ptr[], struct iovec *iov, int idx)
{
	iov[idx].iov_base += shift_len;
	iov_ptr[idx] -= shift_len;
}

static void iov_move_len(int shift_len, int iov_len[], struct iovec *iov, int idx)
{
	iov[idx].iov_len += shift_len;
	iov_len[idx] -= shift_len;
}

static int tx_set_next(int shift_len, int iov_ptr[], int iov_len[], 
		struct iovec *iov, int cnt)
{
	int cur = 0;
	int i, shift;

	for(i = 0; i < cnt; i++){
		if(cur + (int)iov[i].iov_len <= shift_len) {
			cur += iov[i].iov_len;
			iov[i].iov_len = 0;
			continue;
		}
		shift = shift_len - cur;
		iov_move_ptr(shift, iov_ptr, iov, i);
		iov_move_len(-1*shift, iov_len, iov, i);
		return 0;
	}
	fprintf(stderr, "BUG: tx_set_next shift = %d\n", shift_len);
	return -1;
}

static int put_next_pktlen(struct lkl_netdev_fd *nd_fd, int pktlen)
{
	int ret;
	struct iovec txiov;
	uint8_t tmpbuf[TX_HEADER_LEN];

	txiov.iov_len = TX_HEADER_LEN;
	tmpbuf[0] = (pktlen) & 0xff;
	tmpbuf[1] = (pktlen >> 8) & 0xff;
	tmpbuf[2] = (pktlen >> 16) & 0xff;
	tmpbuf[3] = (pktlen >> 24) & 0xff;
	txiov.iov_base = tmpbuf;

	do {
		ret = writev(nd_fd->fd_tx, &txiov, 1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to fd netdev fails header");
		} else {
			char tmp;

			nd_fd->poll_tx = 1;
			if (write(nd_fd->pipe[1], &tmp, 1) <= 0)
				perror("virtio net fd pipe write");
		}
		return ret;
	}
	return ret;
}

static int fd_net_pipe_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);
	int ret = 0;
	int tx_done = 0;
	int iov_ptr[30] = {};
	int iov_len[30] = {};
	int pktlen = get_iovlen(iov, cnt);

	ret = put_next_pktlen(nd_fd, pktlen);
	if(ret < 0)
		return ret;

	while(tx_done < pktlen) {
		do {
			ret = writev(nd_fd->fd_tx, iov, cnt);
		} while (ret == -1 && errno == EINTR);

		if (ret < 0) {
			if (errno != EAGAIN) {
				perror("write to fd netdev fails");
			} 
			continue;
		}
		tx_done += ret;
		if(tx_done == pktlen)
			break;
		tx_set_next(ret, iov_ptr, iov_len, iov, cnt);
	}
	iov_ptr_restore(iov_ptr, iov, cnt);
	iov_len_restore(iov_len, iov, cnt);
	return pktlen;
}

static int fd_net_tx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);

	if(nd_fd->is_pipe)
		return fd_net_pipe_tx(nd, iov, cnt);

	do {
		ret = writev(nd_fd->fd_tx, iov, cnt);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("write to fd netdev fails");
		} else {
			char tmp = 0;

			nd_fd->poll_tx = 1;
			if (write(nd_fd->pipe[1], &tmp, 1) <= 0)
				perror("virtio net fd pipe write");
		}
	}
	return ret;
}

static int rx_adjust_len(int pktlen, int iov_len[], struct iovec *iov, int cnt)
{
	int curlen = 0;
	int i, shift;

	for(i = 0; i < cnt; i++){
		if(curlen >= pktlen){
			shift = iov[i].iov_len;
			iov_move_len(-1*shift, iov_len, iov, i);
		}else if(curlen + (int)iov[i].iov_len > pktlen) {
			shift = iov[i].iov_len - pktlen + curlen;
			iov_move_len(-1*shift, iov_len, iov, i);
			curlen = pktlen;
		}else{
			curlen += iov[i].iov_len;
		}
	}
	if(curlen < pktlen) {
		fprintf(stderr, "BUG: len err curlen = %d pktlen = %d\n", curlen, pktlen);
		return -1;
	}
	return 0;
}

static int rx_set_next(int shift_len, int iov_ptr[], int iov_len[], 
			struct iovec *iov, int cnt)
{
	int cur = 0;
	int i, shift;

	for(i = 0; i < cnt && cur < shift_len; i++){
		if(cur + (int)iov[i].iov_len >= shift_len) {
			shift = shift_len - cur;
		}else{
			shift = iov[i].iov_len;
		}
		cur += shift;
		iov_move_ptr(shift, iov_ptr, iov, i);
		iov_move_len(-1*shift, iov_len, iov, i);
	}

	if(cur < shift_len) {
		fprintf(stderr, "BUG: error in rx_push_iovlen, %d, %d\n", cur, shift_len);
		return -1;
	}

	return 0;
}

static int get_next_pktlen(struct lkl_netdev_fd *nd_fd)
{
	int ret, pktlen;
	struct iovec rxiov;
	uint8_t tmpbuf[TX_HEADER_LEN];
	rxiov.iov_len = TX_HEADER_LEN;
	rxiov.iov_base = tmpbuf;

	do {
		ret = readv(nd_fd->fd_rx, (struct iovec *)&rxiov, 1);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("virtio net fd read");
		} else {
			char tmp;

			nd_fd->poll_rx = 1;
			if (write(nd_fd->pipe[1], &tmp, 1) < 0)
				perror("virtio net fd pipe write");
		}
		return ret;
	}
	pktlen =  ((uint8_t*)rxiov.iov_base)[0];
	pktlen += (((uint8_t*)rxiov.iov_base)[1]<<8);
	pktlen += (((uint8_t*)rxiov.iov_base)[2]<<16);
	pktlen += (((uint8_t*)rxiov.iov_base)[3]<<24);
	return pktlen;
}

static int fd_net_pipe_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);
	int ret = 0;
	int pktlen = 0;
	int rx_done = 0;
	int iov_ptr[30] = {};
	int iov_len[30] = {};

	ret = get_next_pktlen(nd_fd);
	if(ret < 0)
		return ret;
	pktlen = ret;

	if(pktlen > get_iovlen(iov, cnt)){
		/*
		 * discard packet
		 */
		int rest = pktlen;

		while(rest) {
			if(get_iovlen(iov, cnt) > rest){
				rx_adjust_len(rest, iov_len, iov, cnt);
			}
			do {
				ret = readv(nd_fd->fd_rx, (struct iovec *)iov, cnt);
			} while (ret == -1 && errno == EINTR);
			if (ret < 0) {
				if (errno != EAGAIN) {
					perror("write to fd netdev fails");
				} 
				continue;
			}
			rest -= ret;
		}
		iov_ptr_restore(iov_ptr, iov, cnt);
		iov_len_restore(iov_len, iov, cnt);
		return 0;
	}

	rx_adjust_len(pktlen, iov_len, iov, cnt);
	while(rx_done < pktlen) {
		do {
			ret = readv(nd_fd->fd_rx, (struct iovec *)iov, cnt);
		} while (ret == -1 && errno == EINTR);

		if (ret < 0) {
			if (errno != EAGAIN) {
				perror("write to fd netdev fails");
			} 
			/*
			 * use poll because
			 * busy loop decrease the netperf result.
			 */
			struct pollfd pfds[1] = {
				{
					.fd = nd_fd->fd_rx,
				},
			};

			pfds[0].events |= POLLIN|POLLPRI;
			poll(pfds, 1, -1);
			continue;
		}
		rx_done += ret;
		if(rx_done == pktlen){
			break;
		}
		rx_set_next(ret, iov_ptr, iov_len, iov, cnt);
	}
	iov_ptr_restore(iov_ptr, iov, cnt);
	iov_len_restore(iov_len, iov, cnt);
	return pktlen;
}


static int fd_net_rx(struct lkl_netdev *nd, struct iovec *iov, int cnt)
{
	int ret;
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);

	if(nd_fd->is_pipe) 
		return fd_net_pipe_rx(nd, iov, cnt);

	do {
		ret = readv(nd_fd->fd_rx, (struct iovec *)iov, cnt);
	} while (ret == -1 && errno == EINTR);

	if (ret < 0) {
		if (errno != EAGAIN) {
			perror("virtio net fd read");
		} else {
			char tmp = 0;

			nd_fd->poll_rx = 1;
			if (write(nd_fd->pipe[1], &tmp, 1) < 0)
				perror("virtio net fd pipe write");
		}
	}
	return ret;
}

static int fd_net_poll(struct lkl_netdev *nd)
{
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);
	struct pollfd pfds[3] = {
		{
			.fd = nd_fd->fd_rx,
		},
		{
			.fd = nd_fd->fd_tx,
		},
		{
			.fd = nd_fd->pipe[0],
			.events = POLLIN,
		},
	};
	int ret;

	if (nd_fd->poll_rx)
		pfds[0].events |= POLLIN|POLLPRI;
	if (nd_fd->poll_tx)
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

		ret = read(nd_fd->pipe[0], tmp, PIPE_BUF);
		if (ret == 0)
			return LKL_DEV_NET_POLL_HUP;
		if (ret < 0)
			perror("virtio net fd pipe read");
	}

	ret = 0;

	if (pfds[0].revents & (POLLIN|POLLPRI)) {
		nd_fd->poll_rx = 0;
		ret |= LKL_DEV_NET_POLL_RX;
	}

	if (pfds[1].revents & POLLOUT) {
		nd_fd->poll_tx = 0;
		ret |= LKL_DEV_NET_POLL_TX;
	}

	return ret;
}

static void fd_net_poll_hup(struct lkl_netdev *nd)
{
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);

	/* this will cause a POLLHUP / POLLNVAL in the poll function */
	close(nd_fd->pipe[0]);
	close(nd_fd->pipe[1]);
}

static void fd_net_free(struct lkl_netdev *nd)
{
	struct lkl_netdev_fd *nd_fd =
		container_of(nd, struct lkl_netdev_fd, dev);

	close(nd_fd->fd_rx);
	close(nd_fd->fd_tx);
	free(nd_fd);
}

struct lkl_dev_net_ops fd_net_ops =  {
	.tx = fd_net_tx,
	.rx = fd_net_rx,
	.poll = fd_net_poll,
	.poll_hup = fd_net_poll_hup,
	.free = fd_net_free,
};

struct lkl_netdev *lkl_register_netdev_fd(int fd_rx, int fd_tx, int is_pipe)
{
	struct lkl_netdev_fd *nd;

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

	nd->is_pipe = is_pipe;
	nd->dev.ops = &fd_net_ops;
	return &nd->dev;
}
